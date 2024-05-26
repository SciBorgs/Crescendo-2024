package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.sciborgs1155.robot.Constants;

class TalonOdometryThread extends Thread {
  public static final ReadWriteLock lock = new ReentrantReadWriteLock();

  private BaseStatusSignal[] signals = new BaseStatusSignal[0];
  private final List<Queue<Double>> queues = new ArrayList<>();

  private static TalonOdometryThread instance = null;

  public static TalonOdometryThread getInstance() {
    if (instance == null) {
      instance = new TalonOdometryThread();
    }
    return instance;
  }

  @Override
  public void start() {
    super.start();
  }

  public Queue<Double> registerSignal(StatusSignal<Double> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    lock.writeLock().lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, newSignals, 0, signals.length);
      newSignals[signals.length] = signal;
      signals = newSignals;
      queues.add(queue);
    } finally {
      lock.writeLock().unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      BaseStatusSignal.waitForAll(2.0 / Constants.ODOMETRY_PERIOD.in(Seconds), signals);

      lock.writeLock().lock();
      for (int i = 0; i < signals.length; i++) {
        queues.get(i).offer(signals[i].getValueAsDouble());
      }
      lock.writeLock().unlock();
    }
  }
}
