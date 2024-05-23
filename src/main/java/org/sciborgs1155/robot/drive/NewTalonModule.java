package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import java.util.Set;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;

public class NewTalonModule implements ModuleIO {
    private final TalonFX driveMotor;
  private final CANSparkMax turnMotor;

  private final SparkAbsoluteEncoder turnEncoder;

  private final StatusSignal<Double> drivePos;
  private final StatusSignal<Double> driveVelo;

  public NewTalonModule(int drivePort, int turnPort) {
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
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    turnEncoder.setInverted(Turning.ENCODER_INVERTED);
    turnEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians));
    turnEncoder.setVelocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond));

    SparkUtils.configureFrameStrategy(
        turnMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
        Set.of(Sensor.ABSOLUTE),
        false);

    // TODO: TalonFX FaultLogger support (there is code for it somewhere)
    // FaultLogger.register(driveMotor);
    FaultLogger.register(turnMotor);
    turnMotor.burnFlash();
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
