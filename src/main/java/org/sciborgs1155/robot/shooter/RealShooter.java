package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Ports.Shooter.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealShooter implements ShooterIO {
  private final CANSparkFlex topMotor;
  private final CANSparkFlex bottomMotor;
  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  @Log.NT private double setpoint;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  @Log.NT private final PIDController topPID = new PIDController(kP, kI, kD);
  @Log.NT private final PIDController bottomPID = new PIDController(kP, kI, kD);

  public RealShooter() {
    topMotor = new CANSparkFlex(TOP_MOTOR, MotorType.kBrushless);
    topEncoder = topMotor.getEncoder();

    SparkUtils.configure(
        topMotor,
        () ->
            SparkUtils.configureFrameStrategy(
                topMotor,
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.INTEGRATED),
                false),
        () -> topMotor.setIdleMode(IdleMode.kCoast),
        () -> topMotor.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
        () -> SparkUtils.setInverted(topMotor, true),
        () -> topEncoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians)),
        () -> topEncoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond)),
        () -> topEncoder.setAverageDepth(16),
        () -> topEncoder.setMeasurementPeriod(32));

    bottomMotor = new CANSparkFlex(BOTTOM_MOTOR, MotorType.kBrushless);
    bottomEncoder = bottomMotor.getEncoder();
    SparkUtils.configure(
        bottomMotor,
        () ->
            SparkUtils.configureFrameStrategy(
                bottomMotor, Set.of(Data.VELOCITY), Set.of(Sensor.INTEGRATED), false),
        () -> bottomMotor.setIdleMode(IdleMode.kCoast),
        () -> bottomMotor.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
        () -> bottomEncoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians)),
        () -> bottomEncoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond)),
        () -> bottomEncoder.setAverageDepth(16),
        () -> bottomEncoder.setMeasurementPeriod(32));

    FaultLogger.register(topMotor);
    FaultLogger.register(bottomMotor);

    topPID.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));
    bottomPID.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));
  }

  @Override
  public void setSetpoint(double velocity) {
    double ff = feedforward.calculate(setpoint, velocity, PERIOD.in(Seconds));
    double topOut = topPID.calculate(topEncoder.getVelocity(), velocity);
    double bottomOut = bottomPID.calculate(bottomEncoder.getVelocity(), velocity);
    topMotor.setVoltage(MathUtil.clamp(ff + topOut, -12, 12));
    FaultLogger.check(topMotor);
    bottomMotor.setVoltage(MathUtil.clamp(ff + bottomOut, -12, 12));
    FaultLogger.check(bottomMotor);
    setpoint = velocity;
  }

  @Override
  public void setVoltage(double voltage) {
    topMotor.setVoltage(voltage);
    bottomMotor.setVoltage(voltage);
  }

  @Override
  public boolean atSetpoint() {
    return topPID.atSetpoint() && bottomPID.atSetpoint();
  }

  @Log.NT
  @Override
  public double topVelocity() {
    return topEncoder.getVelocity();
  }

  @Log.NT
  @Override
  public double bottomVelocity() {
    return bottomEncoder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    topMotor.close();
    bottomMotor.close();
  }
}
