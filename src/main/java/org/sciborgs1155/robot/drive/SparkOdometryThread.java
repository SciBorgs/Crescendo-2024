package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Constants;

class SparkOdometryThread {
  private final List<DoubleSupplier> signals = new ArrayList<>();
  private final List<Queue<Double>> queues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private final Notifier notifier;

  private static SparkOdometryThread instance = null;

  public static SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  private SparkOdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("odometry-thread");
  }

  public void start() {
    if (timestampQueues.size() > 0) {
      notifier.startPeriodic(1.0 / Constants.ODOMETRY_PERIOD.in(Seconds));
    }
  } // <3 6328

  /**
   * Register a queue that is filled by the polled signal.
   *
   * @param signal
   * @return A queue of doubles that will be populated with signal double values/
   */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> entry = new ArrayBlockingQueue<>(10);
    Drive.lock.writeLock().lock();

    try {
      signals.add(signal);
      queues.add(entry);
    } finally {
      Drive.lock.writeLock().unlock();
    }
    return entry;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    timestampQueues.add(queue);
    return queue;
  }

  private void periodic() {
    Drive.lock.writeLock().lock();
    try {
      double timestamp = Timer.getFPGATimestamp(); // non-deterministic
      for (int i = 0; i < signals.size(); i++) {
        queues.get(i).offer(signals.get(i).getAsDouble());
      }
      for (int i = 0; i < timestampQueues.size(); i++) {
        timestampQueues.get(i).offer(timestamp);
      }
    } finally {
      Drive.lock.writeLock().unlock();
    }
  }
}
