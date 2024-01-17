// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.shooter;

/** May be worth splitting between arm and shooter subsystems */
public interface FlywheelIO extends AutoCloseable {
  public void setFlywheelVoltage();

  public double getRotation();

  public double getFlywheelSpeed();

  public double getRotationSpeed();
}
