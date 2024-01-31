package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

/** Class to encapsulate a CTRE Talon Swerve module */
public class TalonModule implements ModuleIO {
  private final TalonFX driveMotor;
  private final CANSparkMax turnMotor;

  private final SparkAbsoluteEncoder turnEncoder;

  private final MutableMeasure<Distance> drivePos = MutableMeasure.zero(Meters);
  private final MutableMeasure<Velocity<Distance>> driveVelocity =
      MutableMeasure.zero(MetersPerSecond);

  public TalonModule(int drivePort, int turnPort) {
    driveMotor = new TalonFX(drivePort);
    turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
    SparkUtils.configureSettings(false, IdleMode.kBrake, Turning.CURRENT_LIMIT, turnMotor);

    resetEncoders();

    driveMotor.getPosition().setUpdateFrequency(100);
    driveMotor.getVelocity().setUpdateFrequency(100);

    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    turnEncoder.setInverted(Turning.ENCODER_INVERTED);
    turnEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians));
    turnEncoder.setVelocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond));

    SparkUtils.configureFrameStrategy(
        turnMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE),
        Set.of(Sensor.DUTY_CYCLE),
        false);

    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toApply.CurrentLimits.SupplyCurrentLimit = 50;
    driveMotor.getConfigurator().apply(toApply);
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
    return drivePos.mut_replace(driveMotor.getPosition().getValueAsDouble(), Meters);
  }

  @Override
  public Measure<Velocity<Distance>> getDriveVelocity() {
    return driveVelocity.mut_replace(driveMotor.getVelocity().getValueAsDouble(), MetersPerSecond);
  }

  @Override
  public Rotation2d getRotation() {
    return Rotation2d.fromRadians(turnEncoder.getPosition());
  }

  @Override
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  @Override
  public void close() {
    turnMotor.close();
    driveMotor.close();
  }
}
