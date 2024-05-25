package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Constants;

class OdometryThread {
  private List<DoubleSupplier> signals = new ArrayList<>();
  private List<Queue<Double>> queues = new ArrayList<>();

  public static final ReadWriteLock lock = new ReentrantReadWriteLock();

  private final Notifier notifier;

  private static OdometryThread instance = null;

  public static OdometryThread getInstance() {
    if (instance == null) {
      instance = new OdometryThread();
    }
    return instance;
  }

  private OdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("odometry-thread");
  }

  public void start() {
    notifier.startPeriodic(Constants.ODOMETRY_PERIOD.in(Seconds));
  }

  public Queue<Double> registerSignals(DoubleSupplier signal) {
    Queue<Double> entry = new ArrayBlockingQueue<>(10);
    lock.writeLock().lock();

    signals.add(signal);
    queues.add(entry);

    lock.writeLock().unlock();
    return entry;
  }

  private void periodic() {
    lock.writeLock().lock();
    for (int i = 0; i < signals.size(); i++) {
      queues.get(i).offer(signals.get(i).getAsDouble());
    }
    lock.writeLock().unlock();
  }
}
