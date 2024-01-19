package org.sciborgs1155.robot.shooter.flywheel;

import static org.sciborgs1155.robot.Ports.Shooter.Flywheel.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class RealFlywheel implements FlywheelIO {
  private final CANSparkFlex leftMotor;
  private final CANSparkFlex rightMotor;
  private final RelativeEncoder encoder;

  public RealFlywheel() {
    this.leftMotor = new CANSparkFlex(LEFT_MOTOR, MotorType.kBrushless);
    this.rightMotor = new CANSparkFlex(RIGHT_MOTOR, MotorType.kBrushless);
    this.encoder = leftMotor.getEncoder();

    leftMotor.restoreFactoryDefaults();
    leftMotor.setInverted(false);
    leftMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setSmartCurrentLimit(CURRENT_LIMIT);

    rightMotor.restoreFactoryDefaults();
    rightMotor.setInverted(false);
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setSmartCurrentLimit(CURRENT_LIMIT);

    encoder.setVelocityConversionFactor(VELOCITY_CONVERSION);

    rightMotor.follow(leftMotor);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    leftMotor.close();
    rightMotor.close();
  }
}
