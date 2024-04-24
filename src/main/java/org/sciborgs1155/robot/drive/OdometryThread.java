package org.sciborgs1155.robot.drive;

import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

class OdometryThread {
  private List<DoubleSupplier> signals = new ArrayList<>();
  private List<Queue<Double>> queues = new ArrayList<>();

  public static final Lock lock = new ReentrantLock();

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
    notifier.startPeriodic(1.0 / 250.0);
  }

  public Queue<Double> registerSignals(DoubleSupplier signal) {
    Queue<Double> entry = new ArrayBlockingQueue<>(10);
    lock.lock();

    signals.add(signal);
    queues.add(entry);

    lock.unlock();
    return entry;
  }

  private void periodic() {
    lock.lock();
    for (int i = 0; i < signals.size(); i++) {
      queues.get(i).offer(signals.get(i).getAsDouble());
    }
    lock.unlock();
  }
}
