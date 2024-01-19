package org.sciborgs1155.robot.shooter.feeder;

import static org.sciborgs1155.robot.Ports.Shooter.Feeder.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Feeder.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class RealFeeder implements FeederIO {

  private final CANSparkFlex motor;

  public RealFeeder() {
    this.motor = new CANSparkFlex(FEEDER_SPARK, MotorType.kBrushless);

    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);
    motor.setSmartCurrentLimit(CURRENT_LIMIT);

    motor.burnFlash();
  }

  @Override
  public void setVoltage(double speed) {
    motor.setVoltage(speed);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
