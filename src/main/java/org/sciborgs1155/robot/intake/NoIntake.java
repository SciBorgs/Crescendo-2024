package org.sciborgs1155.robot.intake;

import monologue.Annotations.Log;

public class NoIntake implements IntakeIO {
  @Override
  public void power(double percentage) {}

  @Override
  @Log.NT
  public boolean beambreak() {
    return false;
  }

  @Override
  public void close() {}
}
