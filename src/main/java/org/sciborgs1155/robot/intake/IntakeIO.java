package org.sciborgs1155.robot.intake;

import monologue.Logged;

public interface IntakeIO extends AutoCloseable, Logged {
  void setPower(double percentage);

  boolean beambreak();

  double current();

  boolean seenNote();
}
