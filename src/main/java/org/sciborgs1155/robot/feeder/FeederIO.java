package org.sciborgs1155.robot.feeder;

import monologue.Logged;

public interface FeederIO extends AutoCloseable, Logged {
  public void set(double power);

  public boolean beambreak();
}
