package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
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
    driveMotor = new CANSparkFlex(drivePort, MotorType.kBrushless);
    SparkUtils.configureSettings(false, IdleMode.kBrake, Driving.CURRENT_LIMIT, driveMotor);

    turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
    SparkUtils.configureSettings(false, IdleMode.kBrake, Turning.CURRENT_LIMIT, turnMotor);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(Driving.POSITION_FACTOR.in(Radians));
    driveEncoder.setVelocityConversionFactor(Driving.VELOCITY_FACTOR.in(RadiansPerSecond));

    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    turningEncoder.setInverted(Turning.ENCODER_INVERTED);
    turningEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians));
    turningEncoder.setVelocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond));

    SparkUtils.configureFrameStrategy(
        driveMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE),
        Set.of(Sensor.INTEGRATED),
        false);
    SparkUtils.configureFrameStrategy(
        turnMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE),
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
  public void setDriveVoltage(Measure<Voltage> voltage) {
    driveMotor.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setTurnVoltage(Measure<Voltage> voltage) {
    turnMotor.setVoltage(voltage.in(Volts));
  }

  @Override
  public Measure<Distance> getDrivePosition() {
    return Meters.of(driveEncoder.getPosition()).minus(Meters.of(turningEncoder.getPosition()));
    // account for rotation of turn motor on rotation of drive motor
  }

  @Override
  public Measure<Velocity<Distance>> getDriveVelocity() {
    return MetersPerSecond.of(driveEncoder.getVelocity());
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
