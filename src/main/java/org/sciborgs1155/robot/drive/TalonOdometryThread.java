package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import org.sciborgs1155.robot.Constants;

class TalonOdometryThread extends Thread {
  private BaseStatusSignal[] signals = new BaseStatusSignal[0];
  private final List<Queue<Double>> queues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static TalonOdometryThread instance = null;

  public static TalonOdometryThread getInstance() {
    if (instance == null) {
      instance = new TalonOdometryThread();
    }
    return instance;
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  public Queue<Double> registerSignal(StatusSignal<Double> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.lock.writeLock().lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, newSignals, 0, signals.length);
      newSignals[signals.length] = signal;
      signals = newSignals;
      queues.add(queue);
    } finally {
      Drive.lock.writeLock().unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.lock.writeLock().lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.lock.writeLock().lock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      BaseStatusSignal.waitForAll(2.0 * Constants.ODOMETRY_PERIOD.in(Seconds), signals);

      Drive.lock.writeLock().lock();

      try {
        double timestamp = signals[0].getTimestamp().getTime(); // should all be measured together

        double totalLatency = 0.0;
        for (BaseStatusSignal signal : signals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (signals.length > 0) {
          timestamp -= totalLatency / signals.length;
        }
        for (int i = 0; i < signals.length; i++) {
          queues.get(i).offer(signals[i].getValueAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Drive.lock.writeLock().unlock();
      }
    }
  }
}
