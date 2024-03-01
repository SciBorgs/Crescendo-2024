package org.sciborgs1155.robot.intake;

import monologue.Annotations.Log;

public class NoIntake implements IntakeIO {
  @Override
  public void setPower(double percentage) {}

  @Override
  @Log.NT
  public boolean beambreak() {
    return false;
  }

  @Override
  public void close() {}

  @Override
  public double current() {
    return 0;
  }
}
