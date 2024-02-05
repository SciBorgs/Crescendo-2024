package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Ports.Shooter.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealShooter implements ShooterIO {
  private final CANSparkFlex topMotor;
  private final CANSparkFlex bottomMotor;
  private final RelativeEncoder encoder;

  public RealShooter() {

    topMotor = SparkUtils.createSparkFlex(TOP_MOTOR, false, IdleMode.kBrake, CURRENT_LIMIT);
    bottomMotor = SparkUtils.createSparkFlex(BOTTOM_MOTOR, false, IdleMode.kBrake, CURRENT_LIMIT);
    encoder = topMotor.getEncoder();

    encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians));
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond));
    encoder.setMeasurementPeriod(10);
    encoder.setAverageDepth(2);

    SparkUtils.configureFrameStrategy(
        topMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.OUTPUT),
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
