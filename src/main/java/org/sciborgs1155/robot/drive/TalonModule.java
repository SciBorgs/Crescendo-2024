package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.drive.DriveConstants.SENSOR_PERIOD;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class TalonModule implements ModuleIO {
  private final TalonFX driveMotor;
  private final CANSparkMax turnMotor;
  private final SparkAbsoluteEncoder turnEncoder;

  private final SparkPIDController turnPID;
  private final SimpleMotorFeedforward driveFF;

  private final StatusSignal<Double> drivePos;
  private final StatusSignal<Double> driveVelocity;

  private final VelocityVoltage velocityOut = new VelocityVoltage(0);

  private SwerveModuleState setpoint = new SwerveModuleState();

  private final String name;

  public TalonModule(int drivePort, int turnPort, String name) {
    driveMotor = new TalonFX(drivePort);
    drivePos = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveFF =
        new SimpleMotorFeedforward(
            Driving.FF.TALON.S, Driving.FF.TALON.V, Driving.FF.TALON.kA_linear);

    drivePos.setUpdateFrequency(1 / SENSOR_PERIOD.in(Seconds));
    driveVelocity.setUpdateFrequency(1 / SENSOR_PERIOD.in(Seconds));

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    // reset config
    driveMotor.getConfigurator().apply(talonConfig);

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 80;
    talonConfig.Slot0.kP = Driving.PID.TALON.P;
    talonConfig.Slot1.kI = Driving.PID.TALON.I;
    talonConfig.Slot0.kD = Driving.PID.TALON.D;

    driveMotor.getConfigurator().apply(talonConfig);

    TalonUtils.addMotor(driveMotor);

    turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
    turnEncoder = turnMotor.getAbsoluteEncoder();
    turnPID = turnMotor.getPIDController();

    check(turnMotor, turnMotor.restoreFactoryDefaults());

    check(turnMotor, turnPID.setP(Turning.PID.SPARK.P));
    check(turnMotor, turnPID.setI(Turning.PID.SPARK.I));
    check(turnMotor, turnPID.setD(Turning.PID.SPARK.D));
    check(turnMotor, turnPID.setPositionPIDWrappingEnabled(true));
    check(turnMotor, turnPID.setPositionPIDWrappingMinInput(-Math.PI));
    check(turnMotor, turnPID.setPositionPIDWrappingMaxInput(Math.PI));
    check(turnMotor, turnPID.setFeedbackDevice(turnEncoder));

    check(turnMotor, turnMotor.setIdleMode(IdleMode.kBrake));
    check(turnMotor, turnMotor.setSmartCurrentLimit((int) Turning.CURRENT_LIMIT.in(Amps)));
    turnEncoder.setInverted(Turning.ENCODER_INVERTED);
    check(turnMotor);
    check(turnMotor, turnEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians)));
    check(
        turnMotor,
        turnEncoder.setVelocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond)));
    check(turnMotor, turnEncoder.setAverageDepth(2));
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

    register(turnMotor);

    resetEncoders();
    this.name = name;
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
  public String name() {
    return name;
  }

  @Override
  public double drivePosition() {
    return drivePos.getValueAsDouble();
  }

  @Override
  public double driveVelocity() {
    return driveVelocity.getValueAsDouble();
  }

  @Override
  public Rotation2d rotation() {
    return Rotation2d.fromRadians(turnEncoder.getPosition());
  }

  @Override
  public SwerveModuleState state() {
    return new SwerveModuleState(driveVelocity(), rotation());
  }

  @Override
  public SwerveModulePosition position() {
    return new SwerveModulePosition(drivePosition(), rotation());
  }

  @Override
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  @Override
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  @Override
  public void setDriveSetpoint(double velocity) {
    driveMotor.setControl(
        velocityOut.withVelocity(velocity).withFeedForward(driveFF.calculate(velocity)));
  }

  @Override
  public void setTurnSetpoint(double angle) {
    turnPID.setReference(angle, ControlType.kPosition);
  }

  @Override
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {
    setpoint = SwerveModuleState.optimize(setpoint, rotation());
    // Scale setpoint by cos of turning error to reduce tread wear
    setpoint.speedMetersPerSecond *= setpoint.angle.minus(rotation()).getCos();

    if (mode == ControlMode.OPEN_LOOP_VELOCITY) {
      setDriveVoltage(driveFF.calculate(setpoint.speedMetersPerSecond));
    } else {
      setDriveSetpoint(setpoint.speedMetersPerSecond);
    }

    setTurnSetpoint(setpoint.angle.getRadians());
    this.setpoint = setpoint;
  }

  @Override
  public void updateInputs(Rotation2d angle, double voltage) {
    setpoint.angle = angle;
    setDriveVoltage(voltage);
    setTurnSetpoint(angle.getRadians());
  }

  @Override
  public void close() {
    turnMotor.close();
    driveMotor.close();
  }
}
