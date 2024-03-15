package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Set;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

/** Class to encapsulate a CTRE Talon Swerve module */
public class TalonModule implements ModuleIO {
  private final TalonFX driveMotor;
  private final CANSparkMax turnMotor;

  private final SparkAbsoluteEncoder turnEncoder;

  private final StatusSignal<Double> drivePos;
  private final StatusSignal<Double> driveVelo;

  public TalonModule(int drivePort, int turnPort) {
    driveMotor = new TalonFX(drivePort);
    drivePos = driveMotor.getPosition();
    driveVelo = driveMotor.getVelocity();

    drivePos.setUpdateFrequency(100);
    driveVelo.setUpdateFrequency(100);

    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toApply.CurrentLimits.SupplyCurrentLimit = 80;
    driveMotor.getConfigurator().apply(toApply);

    TalonUtils.addMotor(driveMotor);
    resetEncoders();

    turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
    turnEncoder = turnMotor.getAbsoluteEncoder();

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
        () -> turnEncoder.setInverted(Turning.ENCODER_INVERTED),
        () -> turnEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians)),
        () -> turnEncoder.setVelocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond)),
        () -> turnEncoder.setAverageDepth(2));

    FaultLogger.register(driveMotor);
    FaultLogger.register(turnMotor);
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
  public double drivePosition() {
    return drivePos.getValueAsDouble();
  }

  @Override
  public double driveVelocity() {
    return driveVelo.getValueAsDouble();
  }

  @Override
  public Rotation2d rotation() {
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
