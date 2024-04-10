package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.COUPLING_RATIO;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class SparkModule implements ModuleIO {

  private final CANSparkFlex driveMotor; // Neo Vortex
  private final CANSparkMax turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder turningEncoder;

  private final Rotation2d angularOffset;

  private double lastPosition;
  private double lastVelocity;

  /**
   * Constructs a SwerveModule for rev's MAX Swerve.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   */
  public SparkModule(int drivePort, int turnPort, Rotation2d angularOffset) {
    driveMotor = new CANSparkFlex(drivePort, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();

    check(driveMotor, driveMotor.restoreFactoryDefaults());
    check(driveMotor, driveMotor.setIdleMode(IdleMode.kBrake));
    check(driveMotor, driveMotor.setSmartCurrentLimit((int) Driving.CURRENT_LIMIT.in(Amps)));
    check(driveMotor, driveEncoder.setPositionConversionFactor(Driving.POSITION_FACTOR.in(Meters)));
    check(
        driveMotor,
        driveEncoder.setVelocityConversionFactor(Driving.VELOCITY_FACTOR.in(MetersPerSecond)));
    check(driveMotor, driveEncoder.setAverageDepth(16));
    check(driveMotor, driveEncoder.setMeasurementPeriod(32));
    check(
        driveMotor,
        SparkUtils.configureFrameStrategy(
            driveMotor,
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.INTEGRATED),
            false));
    check(driveMotor, driveMotor.burnFlash());

    turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
    turningEncoder = turnMotor.getAbsoluteEncoder();

    check(turnMotor, turnMotor.restoreFactoryDefaults());
    check(turnMotor, turnMotor.setIdleMode(IdleMode.kBrake));
    check(turnMotor, turnMotor.setSmartCurrentLimit((int) Turning.CURRENT_LIMIT.in(Amps)));
    turningEncoder.setInverted(Turning.ENCODER_INVERTED);
    check(turnMotor);
    check(
        turnMotor, turningEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians)));
    check(
        turnMotor,
        turningEncoder.setVelocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond)));
    check(turnMotor, turningEncoder.setAverageDepth(2));
    check(
        turnMotor,
        SparkUtils.configureFrameStrategy(
            turnMotor,
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.ABSOLUTE),
            false));
    SparkUtils.addChecker(
        () ->
            check(
                turnMotor,
                SparkUtils.configureFrameStrategy(
                    turnMotor,
                    Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                    Set.of(Sensor.ABSOLUTE),
                    false)));
    check(turnMotor, turnMotor.burnFlash());

    register(driveMotor);
    register(turnMotor);

    resetEncoders();

    this.angularOffset = angularOffset;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
    check(driveMotor);
    log("current", driveMotor.getOutputCurrent());
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
    check(turnMotor);
  }

  @Override
  public double drivePosition() {
    lastPosition = SparkUtils.wrapCall(driveMotor, driveEncoder.getPosition()).orElse(lastPosition);
    // account for rotation of turn motor on rotation of drive motor
    return lastPosition - turningEncoder.getPosition() * COUPLING_RATIO;
  }

  @Override
  public double driveVelocity() {
    lastVelocity = SparkUtils.wrapCall(driveMotor, driveEncoder.getVelocity()).orElse(lastVelocity);
    return lastVelocity;
  }

  @Override
  public Rotation2d rotation() {
    return Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset);
  }

  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void close() {
    driveMotor.close();
    turnMotor.close();
  }
}
