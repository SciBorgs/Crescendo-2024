package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.List;
import java.util.function.Supplier;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.vision.Vision.PoseEstimate;

public class Drive extends SubsystemBase implements Logged, AutoCloseable {

  // Modules
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule rearLeft;
  private final SwerveModule rearRight;

  @IgnoreLogged private final List<SwerveModule> modules;

  private final GyroIO gyro;
  private static Rotation2d simRotation = new Rotation2d();

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_OFFSET);

  // Odometry and pose estimation
  private final SwerveDrivePoseEstimator odometry;

  @Log.NT private final Field2d field2d = new Field2d();
  private final FieldObject2d[] modules2d;

  // SysId
  private final SysIdRoutine driveRoutine;
  private final SysIdRoutine turnRoutine;

  @Log.NT
  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(
          Rotation.P,
          Rotation.I,
          Rotation.D,
          new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL));

  /**
   * A factory to create a new drive subsystem based on whether the robot is being ran in simulation
   * or not.
   */
  public static Drive create() {
    return Robot.isReal()
        ? new Drive(
            new GyroIO.NavX(),
            new FlexModule(FRONT_LEFT_DRIVE, FRONT_LEFT_TURNING, ANGULAR_OFFSETS.get(0)),
            new FlexModule(FRONT_RIGHT_DRIVE, FRONT_RIGHT_TURNING, ANGULAR_OFFSETS.get(1)),
            new FlexModule(REAR_LEFT_DRIVE, REAR_LEFT_TURNING, ANGULAR_OFFSETS.get(2)),
            new FlexModule(REAR_RIGHT_DRIVE, REAR_RIGHT_TURNING, ANGULAR_OFFSETS.get(3)))
        : new Drive(
            new GyroIO.NoGyro(),
            new SimModule(),
            new SimModule(),
            new SimModule(),
            new SimModule());
  }

  /** A swerve drive subsystem containing four {@link ModuleIO} modules. */
  public Drive(
      GyroIO gyro, ModuleIO frontLeft, ModuleIO frontRight, ModuleIO rearLeft, ModuleIO rearRight) {
    this.gyro = gyro;
    this.frontLeft = new SwerveModule(frontLeft, ANGULAR_OFFSETS.get(0), " FL");
    this.frontRight = new SwerveModule(frontRight, ANGULAR_OFFSETS.get(1), "FR");
    this.rearLeft = new SwerveModule(rearLeft, ANGULAR_OFFSETS.get(2), "RL");
    this.rearRight = new SwerveModule(rearRight, ANGULAR_OFFSETS.get(3), " RR");

    modules = List.of(this.frontLeft, this.frontRight, this.rearLeft, this.rearRight);
    modules2d = new FieldObject2d[modules.size()];

    driveRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                volts -> modules.forEach(m -> m.setDriveVoltage(volts.in(Volts))),
                null,
                this,
                "drive routine"));

    turnRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // volts -> modules.forEach(m -> m.setTurnVoltage(volts.in(Volts))),
                volts -> modules.forEach(m -> m.setTurnVoltage(volts.in(Volts))),
                null,
                this,
                "turn routine"));

    odometry =
        new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePositions(), new Pose2d());

    for (int i = 0; i < modules.size(); i++) {
      var module = modules.get(i);
      modules2d[i] = field2d.getObject("module-" + module.name);
    }

    rotationController.enableContinuousInput(0, 2 * Math.PI);

    SmartDashboard.putData("drive quasistatic forward", driveSysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData("drive dynamic forward", driveSysIdDynamic(Direction.kForward));
    SmartDashboard.putData("drive quasistatic backward", driveSysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData("drive dynamic backward", driveSysIdDynamic(Direction.kReverse));
    SmartDashboard.putData("turn quasistatic forward", turnSysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData("turn dynamic forward", turnSysIdDynamic(Direction.kForward));
    SmartDashboard.putData("turn quasistatic backward", turnSysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData("turn dynamic backward", turnSysIdDynamic(Direction.kReverse));
    SmartDashboard.putData("face same direction", alignModuleDirections());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @Log.NT
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  @Log.NT
  public Rotation2d getHeading() {
    return gyro.getRotation2d();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /**
   * Drives the robot while facing a target pose.
   *
   * @param vx A supplier for the absolute x velocity of the robot.
   * @param vy A supplier for the absolute y velocity of the robot.
   * @param translation A supplier for the translation2d to face on the field.
   * @return A command to drive while facing a target.
   */
  public Command driveFacingTarget(
      InputStream vx, InputStream vy, Supplier<Translation2d> translation) {
    return drive(vx, vy, () -> translation.get().minus(getPose().getTranslation()).getAngle());
  }

  /**
   * Drives the robot based on a {@link InputStream} for field relative x y and omega velocities.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param vOmega A supplier for the angular velocity of the robot.
   * @return The driving command.
   */
  public Command drive(InputStream vx, InputStream vy, InputStream vOmega) {
    return run(() -> driveFieldRelative(new ChassisSpeeds(vx.get(), vy.get(), vOmega.get())));
  }

  /**
   * Drives the robot based on a {@link InputStream} for field relative x y and omega velocities.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param heading A supplier for the field relative heading of the robot.
   * @return The driving command.
   */
  public Command drive(InputStream vx, InputStream vy, Supplier<Rotation2d> heading) {
    return run(
        () ->
            driveFieldRelative(
                new ChassisSpeeds(
                    vx.get(),
                    vy.get(),
                    rotationController.calculate(
                        getPose().getRotation().getRadians(), heading.get().getRadians()))));
  }

  public Command goTo(Pose2d targetPose) {
    // store current position to calculate nessesary components
    Pose2d initialPose = getPose();

    // calculates components of 3D vector
    double deltaX = targetPose.getX() - initialPose.getX();
    double deltaY = targetPose.getY() - initialPose.getY();
    double deltaTheta =
        targetPose.getRotation().getRadians() - initialPose.getRotation().getRadians();

    // calulates magninutde of vector
    // sqrt((x)^2 + (y)^2 + (w*r)^2)
    double magnitude =
        Math.sqrt(
            Math.pow(deltaX, 2)
                + Math.pow(deltaY, 2)
                + Math.pow(deltaTheta * DriveToConstants.ANGULAR_RADIUS, 2));

    var pid =
        new ProfiledPIDController(
            DriveToConstants.PID.P,
            DriveToConstants.PID.I,
            DriveToConstants.PID.D,
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL));
    double output = pid.calculate(magnitude);
    double vx, vy, theta;
    // Scaling every individual component
    vx = output * (deltaX / magnitude);
    vy = output * (deltaY / magnitude);
    theta = output * (deltaTheta / magnitude);
    return run(() -> driveFieldRelative(new ChassisSpeeds(vx, vy, theta)));
  }

  /**
   * Drives the robot relative to field based on provided {@link ChassisSpeeds} and current heading.
   *
   * @param speeds The desired field relative chassis speeds.
   */
  public void driveFieldRelative(ChassisSpeeds speeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()));
  }

  /**
   * Drives the robot based on profided {@link ChassisSpeeds}.
   *
   * <p>This method uses {@link ChassisSpeeds#discretize(ChassisSpeeds, double)} to reduce skew.
   *
   * @param speeds The desired robot relative chassis speeds.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    setModuleStates(
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(speeds, Constants.PERIOD.in(Seconds))));
  }

  public void configureAuto() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeed,
        this::driveRobotRelative,
        new HolonomicPathFollowerConfig(
            new PIDConstants(Translation.P, Translation.I, Translation.D),
            new PIDConstants(Rotation.P, Rotation.I, Rotation.D),
            MAX_SPEED.in(MetersPerSecond),
            TRACK_WIDTH.divide(2).in(Meters),
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          throw new RuntimeException("ahhhhhhhhhh");
        },
        this);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    if (desiredStates.length != modules.size()) {
      throw new IllegalArgumentException("desiredStates must have the same length as modules");
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED.in(MetersPerSecond));

    for (int i = 0; i < modules.size(); i++) {
      modules.get(i).updateDesiredState(desiredStates[i]);
    }
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    modules.forEach(SwerveModule::resetEncoders);
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeading() {
    return runOnce(gyro::reset);
  }

  /** Returns the module states. */
  @Log.NT
  public SwerveModuleState[] getModuleStates() {
    return modules.stream().map(SwerveModule::state).toArray(SwerveModuleState[]::new);
  }

  /** Returns the module states. */
  @Log.NT
  private SwerveModuleState[] getModuleSetpoints() {
    return modules.stream().map(SwerveModule::desiredState).toArray(SwerveModuleState[]::new);
  }

  /** Returns the module positions */
  @Log.NT
  public SwerveModulePosition[] getModulePositions() {
    return modules.stream().map(SwerveModule::position).toArray(SwerveModulePosition[]::new);
  }

  /** Returns the chassis speed. */
  @Log.NT
  public ChassisSpeeds getChassisSpeed() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Updates pose estimation based on provided {@link EstimatedRobotPose} */
  public void updateEstimates(PoseEstimate... poses) {
    for (int i = 0; i < poses.length; i++) {
      odometry.addVisionMeasurement(
          poses[i].estimatedPose().estimatedPose.toPose2d(),
          poses[i].estimatedPose().timestampSeconds,
          poses[i].standardDev());
      field2d
          .getObject("Cam " + i + " Est Pose")
          .setPose(poses[i].estimatedPose().estimatedPose.toPose2d());
    }
  }

  @Override
  public void periodic() {
    odometry.update(Robot.isReal() ? getHeading() : simRotation, getModulePositions());

    field2d.setRobotPose(getPose());

    for (int i = 0; i < modules2d.length; i++) {
      var module = modules.get(i);
      var transform = new Transform2d(MODULE_OFFSET[i], module.position().angle);
      modules2d[i].setPose(getPose().transformBy(transform));
    }
  }

  @Override
  public void simulationPeriodic() {
    simRotation =
        simRotation.rotateBy(
            Rotation2d.fromRadians(
                getChassisSpeed().omegaRadiansPerSecond * Constants.PERIOD.in(Seconds)));
  }

  /** Stops drivetrain */
  public Command stop() {
    return runOnce(() -> driveRobotRelative(new ChassisSpeeds()));
  }

  /** Sets the drivetrain to an "X" configuration, preventing movement */
  public Command lock() {
    var front = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    var back = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    return run(() -> setModuleStates(new SwerveModuleState[] {front, back, back, front}));
  }

  /** Locks the drive motors. */
  private Command lockDriveMotors() {
    return Commands.run(() -> modules.forEach(m -> m.updateDriveSpeed(0)));
  }

  /** Locks the turn motors. */
  private Command lockTurnMotors() {
    return Commands.run(
        () -> modules.forEach(m -> m.updateTurnRotation(Rotation2d.fromDegrees(0))));
  }

  private Command alignModuleDirections() {
    return Commands.run(
        () -> modules.forEach(m -> m.updateTurnRotation(ANGULAR_OFFSETS.get(modules.indexOf(m)))));
  }

  /** Runs the drive quasistatic SysId while locking turn motors. */
  public Command driveSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return driveRoutine.quasistatic(direction).deadlineWith(lockTurnMotors());
  }

  /** Runs the drive dynamic SysId while locking turn motors. */
  public Command driveSysIdDynamic(SysIdRoutine.Direction direction) {
    return driveRoutine.dynamic(direction).deadlineWith(lockTurnMotors());
  }

  /** Runs the turn quasistatic SysId while locking drive motors. */
  public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return (turnRoutine.quasistatic(direction).deadlineWith(lockDriveMotors()));
  }

  /** Runs the turn dynamic SysId while locking drive motors. */
  public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
    return (turnRoutine.dynamic(direction).deadlineWith(lockDriveMotors()));
  }

  public void close() throws Exception {
    frontLeft.close();
    frontRight.close();
    rearLeft.close();
    rearRight.close();
  }
}
