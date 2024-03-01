package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.COUPLING_RATIO;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Set;
import org.sciborgs1155.lib.FaultLogger;
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

  /**
   * Constructs a SwerveModule for rev's MAX Swerve.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   */
  public SparkModule(int drivePort, int turnPort, Rotation2d angularOffset) {
    driveMotor = new CANSparkFlex(drivePort, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();

    SparkUtils.configure(
        driveMotor,
        () ->
            SparkUtils.configureFrameStrategy(
                driveMotor,
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.INTEGRATED),
                false),
        () -> driveMotor.setIdleMode(IdleMode.kBrake),
        () -> driveMotor.setSmartCurrentLimit((int) Driving.CURRENT_LIMIT.in(Amps)),
        () -> driveEncoder.setPositionConversionFactor(Driving.POSITION_FACTOR.in(Meters)),
        () -> driveEncoder.setVelocityConversionFactor(Driving.VELOCITY_FACTOR.in(MetersPerSecond)),
        () -> driveEncoder.setMeasurementPeriod(4),
        () -> driveEncoder.setAverageDepth(2));

    turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
    turningEncoder = turnMotor.getAbsoluteEncoder();

    SparkUtils.configure(
        turnMotor,
        () ->
            SparkUtils.configureFrameStrategy(
                turnMotor,
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.ABSOLUTE),
                false),
        () -> turnMotor.setIdleMode(IdleMode.kBrake),
        () -> turnMotor.setSmartCurrentLimit((int) Turning.CURRENT_LIMIT.in(Amps)),
        () -> turningEncoder.setInverted(Turning.ENCODER_INVERTED),
        () -> turningEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians)),
        () ->
            turningEncoder.setVelocityConversionFactor(
                Turning.VELOCITY_FACTOR.in(RadiansPerSecond)),
        () -> turningEncoder.setAverageDepth(2));

    FaultLogger.register(driveMotor);
    FaultLogger.register(turnMotor);

    resetEncoders();

    this.angularOffset = angularOffset;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
    FaultLogger.check(driveMotor);
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
    FaultLogger.check(turnMotor);
  }

  @Override
  public double drivePosition() {
    double driveRot = driveEncoder.getPosition();
    // account for rotation of turn motor on rotation of drive motor
    driveRot -= turningEncoder.getPosition() * COUPLING_RATIO;
    return driveRot;
  }

  @Override
  public double driveVelocity() {
    return driveEncoder.getVelocity();
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
