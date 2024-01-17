// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.shooter;

import static org.sciborgs1155.robot.Ports.Shooter.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class RealFlywheel {
  private final CANSparkFlex flywheel = new CANSparkFlex(FLYWHEEL, MotorType.kBrushless);
  private final SparkENcod
  
  public RealFlywheel() {

  }
}
