package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.Ports.Shooter.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import java.util.Set;
import monologue.Annotations.Log;
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

    check(topMotor, topMotor.restoreFactoryDefaults());
    SparkUtils.configureFrameStrategy(
        topMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
        Set.of(Sensor.INTEGRATED),
        true);
    check(topMotor, topMotor.setIdleMode(IdleMode.kCoast));
    check(topMotor, topMotor.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));
    topMotor.setInverted(true);
    check(topMotor);
    check(topMotor, encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians)));
    check(topMotor, encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond)));
    check(topMotor, encoder.setAverageDepth(16));
    check(topMotor, encoder.setMeasurementPeriod(32));
    check(topMotor, topMotor.burnFlash());

    bottomMotor = new CANSparkFlex(BOTTOM_MOTOR, MotorType.kBrushless);
    check(bottomMotor, bottomMotor.restoreFactoryDefaults());
    check(bottomMotor, SparkUtils.configureNothingFrameStrategy(bottomMotor));
    check(bottomMotor, bottomMotor.setIdleMode(IdleMode.kCoast));
    check(bottomMotor, bottomMotor.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));
    check(bottomMotor, bottomMotor.follow(topMotor, true));
    check(bottomMotor, bottomMotor.burnFlash());

    register(topMotor);
    register(bottomMotor);
  }

  @Override
  public void setVoltage(double voltage) {
    topMotor.setVoltage(voltage);
    check(topMotor);
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
