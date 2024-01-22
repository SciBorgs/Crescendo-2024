package org.sciborgs1155.robot.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Shooter.Flywheel.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel.*;

import java.util.Set;

import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

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
    leftMotor.setSmartCurrentLimit((int) Math.round(CURRENT_LIMIT.in(Amps)));

    rightMotor.restoreFactoryDefaults();
    rightMotor.setInverted(false);
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setSmartCurrentLimit((int) Math.round(CURRENT_LIMIT.in(Amps)));

    encoder.setVelocityConversionFactor(VELOCITY_CONVERSION);
    encoder.setPositionConversionFactor(POSITION_CONVERSION);

    SparkUtils.configureFrameStrategy(
      leftMotor, 
      Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE),
      Set.of(Sensor.INTEGRATED),
      true
      );
    SparkUtils.configureFollowerFrameStrategy(
      rightMotor
      );

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
