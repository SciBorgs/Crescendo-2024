// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.shooter.pivot;

/** Add your docs here. */
public interface PivotIO {
  // Sets voltage to the motor
  public void setVoltage(double voltage);

  public double getPosition();
}
