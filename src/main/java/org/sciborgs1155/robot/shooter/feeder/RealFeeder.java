package org.sciborgs1155.robot.shooter.feeder;

import static org.sciborgs1155.robot.Ports.Shooter.Feeder.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class RealFeeder implements FeederIO {

  private final CANSparkFlex motor = new CANSparkFlex(FEEDER_SPARK, MotorType.kBrushless);

  public RealFeeder() {
    motor.setIdleMode(IdleMode.kBrake);
  }

  // @Override
  // public double getFeederVoltage() {
  //     return motor.getBusVoltage();
  // }

  @Override
  public void setFeederVoltage(double speed) {
    motor.setVoltage(speed);
  }
}
