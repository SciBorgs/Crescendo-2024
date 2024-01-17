package org.sciborgs1155.robot.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.List;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Turning;

/** Class to encapsulate a rev max swerve module */
public class MAXSwerveModule implements ModuleIO {

  private final CANSparkMax driveMotor; // Regular Neo
  private final CANSparkMax turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController driveFeedback;
  private final SparkMaxPIDController turnFeedback;

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.FF.s(), Driving.FF.v(), Driving.FF.a());

  private final String name;
  private final Rotation2d angularOffset;

  private SwerveModuleState setpoint = new SwerveModuleState();

  /**
   * Constructs a SwerveModule for rev's MAX Swerve.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param angularOffset offset from drivetrain
   */
  public MAXSwerveModule(String name, int drivePort, int turnPort, double angularOffset) {
    this.name = name;

    driveMotor = SparkUtils.create(drivePort);
    turnMotor = SparkUtils.create(turnPort);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    driveFeedback = driveMotor.getPIDController();
    turnFeedback = turnMotor.getPIDController();

    driveFeedback.setFeedbackDevice(driveEncoder);
    turnFeedback.setFeedbackDevice(turningEncoder);

    turningEncoder.setInverted(Turning.ENCODER_INVERTED);

    setDrivePID(Driving.PID);
    setTurnPID(Turning.PID);

    driveEncoder.setPositionConversionFactor(Driving.CONVERSION);
    driveEncoder.setVelocityConversionFactor(Driving.CONVERSION / 60.0);
    turningEncoder.setPositionConversionFactor(Turning.CONVERSION);
    turningEncoder.setVelocityConversionFactor(Turning.CONVERSION / 60.0);

    // set up continuous input for turning
    turnFeedback.setPositionPIDWrappingEnabled(true);
    turnFeedback.setPositionPIDWrappingMinInput(0);
    turnFeedback.setPositionPIDWrappingMaxInput(Turning.CONVERSION);

    SparkUtils.disableFrames(driveMotor, 4, 5, 6);
    SparkUtils.disableFrames(turnMotor, 4, 6);

    driveEncoder.setPosition(0);
    this.angularOffset = Rotation2d.fromRadians(angularOffset);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(),
        Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(angularOffset);
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint =
        SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d.fromRadians(turningEncoder.getPosition()));

    double driveFF = driveFeedforward.calculate(setpoint.speedMetersPerSecond);
    driveFeedback.setReference(setpoint.speedMetersPerSecond, ControlType.kVelocity, 0, driveFF);
    turnFeedback.setReference(setpoint.angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return setpoint;
  }

  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void setTurnPID(PIDConstants constants) {
    turnFeedback.setP(constants.p());
    turnFeedback.setI(constants.i());
    turnFeedback.setD(constants.d());
  }

  @Override
  public void setDrivePID(PIDConstants constants) {
    driveFeedback.setP(constants.p());
    driveFeedback.setI(constants.i());
    driveFeedback.setD(constants.d());
  }

  @Override
  public List<HardwareFault> getFaults() {
    return FaultBuilder.create()
        .register(name + " drive", driveMotor)
        .register(name + " turn", turnMotor)
        .build();
  }

  @Override
  public void close() {
    driveMotor.close();
    turnMotor.close();
  }
}
