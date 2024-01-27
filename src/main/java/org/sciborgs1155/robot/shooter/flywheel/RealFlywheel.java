package org.sciborgs1155.robot.shooter.flywheel;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Ports.Shooter.Flywheel.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.FlywheelConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealFlywheel implements FlywheelIO {
  private final CANSparkFlex topMotor;
  private final CANSparkFlex bottomMotor;
  private final RelativeEncoder encoder;

  public RealFlywheel() {

    topMotor = new CANSparkFlex(LEFT_MOTOR, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(RIGHT_MOTOR, MotorType.kBrushless);
    encoder = topMotor.getEncoder();

    SparkUtils.configureSettings(false, IdleMode.kBrake, CURRENT_LIMIT, topMotor);
    SparkUtils.configureSettings(true, IdleMode.kBrake, CURRENT_LIMIT, bottomMotor);

    encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians));
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond));

    SparkUtils.configureFrameStrategy(
        topMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE),
        Set.of(Sensor.INTEGRATED),
        true);
    SparkUtils.configureFollowerFrameStrategy(bottomMotor);

    bottomMotor.follow(topMotor);

    topMotor.burnFlash();
    bottomMotor.burnFlash();
  }

  @Override
  public void setVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    topMotor.close();
    bottomMotor.close();
  }
}
