package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.COUPLING_RATIO;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Set;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class FlexModule implements ModuleIO {

  private final CANSparkBase driveMotor; // Neo Vortex
  private final CANSparkBase turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder turningEncoder;

  private final Rotation2d angularOffset;

  /**
   * Constructs a SwerveModule for rev's MAX Swerve.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   */
  public FlexModule(int drivePort, int turnPort, Rotation2d angularOffset) {
    driveMotor =
        SparkUtils.createSparkFlex(drivePort, false, IdleMode.kBrake, Driving.CURRENT_LIMIT);
    turnMotor = SparkUtils.createSparkMax(turnPort, false, IdleMode.kBrake, Turning.CURRENT_LIMIT);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(Driving.POSITION_FACTOR.in(Radians));
    driveEncoder.setVelocityConversionFactor(Driving.VELOCITY_FACTOR.in(RadiansPerSecond));

    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    turningEncoder.setInverted(Turning.ENCODER_INVERTED);
    turningEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians));
    turningEncoder.setVelocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond));

    SparkUtils.configureFrameStrategy(
        driveMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.OUTPUT),
        Set.of(Sensor.INTEGRATED),
        false);
    SparkUtils.configureFrameStrategy(
        turnMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.OUTPUT),
        Set.of(Sensor.DUTY_CYCLE),
        false);

    FaultLogger.register(driveMotor);
    FaultLogger.register(turnMotor);

    driveMotor.burnFlash();
    turnMotor.burnFlash();

    resetEncoders();

    this.angularOffset = angularOffset;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
  }

  @Override
  public double getDrivePosition() {
    double driveRot = driveEncoder.getPosition();
    // account for rotation of turn motor on rotation of drive motor
    driveRot -= turningEncoder.getPosition() * COUPLING_RATIO;
    return driveRot;
  }

  @Override
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  @Override
  public Rotation2d getRotation() {
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
