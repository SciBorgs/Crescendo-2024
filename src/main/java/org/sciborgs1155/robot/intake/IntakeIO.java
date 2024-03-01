package org.sciborgs1155.robot.intake;

import monologue.Logged;

public interface IntakeIO extends AutoCloseable, Logged {
  void power(double percentage);

  boolean beambreak();
}
