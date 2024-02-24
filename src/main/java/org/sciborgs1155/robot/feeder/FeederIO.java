package org.sciborgs1155.robot.feeder;

import monologue.Annotations.Log;
import monologue.Logged;

public interface FeederIO extends AutoCloseable, Logged {
  public void set(double power);

  public double getVelocity();

  @Log.NT
  public boolean beambreak();
}
