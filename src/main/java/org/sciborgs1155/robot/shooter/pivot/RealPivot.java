// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.shooter.pivot;

import static org.sciborgs1155.robot.Ports.Shooter.Pivot.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Pivot.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class RealPivot implements PivotIO {
  private final CANSparkFlex motor;
  private final DutyCycleEncoder encoder;

  public RealPivot() {
    this.motor = new CANSparkFlex(PIVOT_SPARK_ONE, MotorType.kBrushless);
    this.encoder = new DutyCycleEncoder(PIVOT_THROUGHBORE);

    encoder.setDistancePerRotation(CONVERSION);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return encoder.getAbsolutePosition();
  }
}
