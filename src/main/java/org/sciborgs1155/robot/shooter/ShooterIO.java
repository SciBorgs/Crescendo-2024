package org.sciborgs1155.robot.shooter;

import monologue.Logged;

public interface ShooterIO extends AutoCloseable, Logged {
  void setSetpoint(double velocity);

  void setVoltage(double voltage);

  double topVelocity();

  double bottomVelocity();

  boolean atSetpoint();
}
