package org.sciborgs1155.robot.feeder;

import monologue.Logged;

public interface FeederIO extends AutoCloseable, Logged {
  void set(double power);

  double current();

  boolean beambreak();
}
