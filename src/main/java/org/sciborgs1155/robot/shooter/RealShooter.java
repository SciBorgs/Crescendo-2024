package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Ports.Shooter.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealShooter implements ShooterIO {
  private final CANSparkFlex topMotor;
  private final CANSparkFlex bottomMotor;
  private final RelativeEncoder encoder;

  public RealShooter() {
    topMotor = new CANSparkFlex(TOP_MOTOR, MotorType.kBrushless);
    encoder = topMotor.getEncoder();

    SparkUtils.configureFrameStrategy(
        topMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
        Set.of(Sensor.INTEGRATED),
        true);
    FaultLogger.check(topMotor);
    topMotor.setIdleMode(IdleMode.kCoast);
    FaultLogger.check(topMotor);
    topMotor.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    FaultLogger.check(topMotor);
    SparkUtils.setInverted(topMotor, true);
    FaultLogger.check(topMotor);
    encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians));
    FaultLogger.check(topMotor);
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond));
    FaultLogger.check(topMotor);
    encoder.setAverageDepth(16);
    FaultLogger.check(topMotor);
    encoder.setMeasurementPeriod(32);
    FaultLogger.check(topMotor);

    bottomMotor = new CANSparkFlex(BOTTOM_MOTOR, MotorType.kBrushless);
    SparkUtils.configureNothingFrameStrategy(bottomMotor);
    FaultLogger.check(bottomMotor);
    bottomMotor.setIdleMode(IdleMode.kCoast);
    FaultLogger.check(bottomMotor);
    bottomMotor.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    FaultLogger.check(bottomMotor);
    bottomMotor.follow(topMotor, true);
    FaultLogger.check(bottomMotor);

    FaultLogger.register(topMotor);
    FaultLogger.register(bottomMotor);
  }

  @Override
  public void setVoltage(double voltage) {
    topMotor.setVoltage(voltage);
    FaultLogger.check(topMotor);
  }

  @Override
  @Log.NT
  public double current() {
    return topMotor.getOutputCurrent();
  }

  @Override
  public double velocity() {
    return encoder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    topMotor.close();
    bottomMotor.close();
  }
}
