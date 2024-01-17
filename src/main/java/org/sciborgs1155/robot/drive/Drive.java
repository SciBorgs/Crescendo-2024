package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.lib.PathFlipper.pathForAlliance;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.photonvision.EstimatedRobotPose;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.lib.failure.FaultBuilder;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

public class Drive extends SubsystemBase implements Fallible, Loggable, AutoCloseable {

  @Log private final ModuleIO frontLeft;
  @Log private final ModuleIO frontRight;
  @Log private final ModuleIO rearLeft;
  @Log private final ModuleIO rearRight;

  private final List<ModuleIO> modules;

  // this should be a generic IMU class, once WPILib implements it
  @Log private final WPI_PigeonIMU imu = new WPI_PigeonIMU(PIGEON);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_OFFSET);

  // Odometry and pose estimation
  private final SwerveDrivePoseEstimator odometry;

  @Log private final Field2d field2d = new Field2d();
  private final FieldObject2d[] modules2d;

  // Rate limiting
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(MAX_ACCEL);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(MAX_ACCEL);

  @Log private double speedMultiplier = 1;

  public static Drive create() {
    return Robot.isReal()
        ? new Drive(
            new MAXSwerveModule(
                MODULE_NAMES[0], FRONT_LEFT_DRIVE, FRONT_LEFT_TURNING, ANGULAR_OFFSETS[0]),
            new MAXSwerveModule(
                MODULE_NAMES[1], FRONT_RIGHT_DRIVE, FRONT_RIGHT_TURNING, ANGULAR_OFFSETS[1]),
            new MAXSwerveModule(
                MODULE_NAMES[2], REAR_LEFT_DRIVE, REAR_LEFT_TURNING, ANGULAR_OFFSETS[2]),
            new MAXSwerveModule(
                MODULE_NAMES[3], REAR_RIGHT_DRIVE, REAR_RIGHT_TURNING, ANGULAR_OFFSETS[3]))
        : new Drive(new SimModule(), new SimModule(), new SimModule(), new SimModule());
  }

  public Drive(ModuleIO frontLeft, ModuleIO frontRight, ModuleIO rearLeft, ModuleIO rearRight) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearLeft = rearLeft;
    this.rearRight = rearRight;

    modules = List.of(frontLeft, frontRight, rearLeft, rearRight);
    modules2d = new FieldObject2d[modules.size()];

    odometry =
        new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePositions(), new Pose2d());

    for (int i = 0; i < modules2d.length; i++) {
      modules2d[i] = field2d.getObject("module-" + i);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Returns the heading of the robot, based on our pigeon
   *
   * @return A Rotation2d of our angle
   */
  public Rotation2d getHeading() {
    return imu.getRotation2d();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /** Deadbands and squares inputs */
  private static double scale(double input) {
    input = MathUtil.applyDeadband(input, Constants.DEADBAND);
    return Math.copySign(input * input, input);
  }

  /** Drives the robot based on a {@link DoubleSupplier} for x y and omega velocities */
  public CommandBase drive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vOmega) {
    return run(
        () ->
            drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xLimiter.calculate(scale(vx.getAsDouble()) * MAX_SPEED * speedMultiplier),
                    yLimiter.calculate(scale(vy.getAsDouble()) * MAX_SPEED * speedMultiplier),
                    scale(vOmega.getAsDouble()) * MAX_ANGULAR_SPEED * speedMultiplier,
                    getHeading())));
  }

  /**
   * Drives the robot based on profided {@link ChassisSpeeds}.
   *
   * <p>This method uses {@link Pose2d#log(Pose2d)} to reduce skew.
   *
   * @param speeds The desired chassis speeds.
   */
  public void drive(ChassisSpeeds speeds) {
    var target =
        new Pose2d(
            speeds.vxMetersPerSecond * Constants.PERIOD,
            speeds.vyMetersPerSecond * Constants.PERIOD,
            Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Constants.PERIOD));

    var twist = new Pose2d().log(target);

    speeds =
        new ChassisSpeeds(
            twist.dx / Constants.PERIOD,
            twist.dy / Constants.PERIOD,
            twist.dtheta / Constants.PERIOD);

    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired ModuleIO states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    if (desiredStates.length != modules.size()) {
      throw new IllegalArgumentException("desiredStates must have the same length as modules");
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

    for (int i = 0; i < modules.size(); i++) {
      modules.get(i).setDesiredState(desiredStates[i]);
    }
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    modules.forEach(ModuleIO::resetEncoders);
  }

  /** Zeroes the heading of the robot. */
  public CommandBase zeroHeading() {
    return runOnce(imu::reset);
  }

  /** Returns the pitch of the drive gyro */
  public double getPitch() {
    return imu.getPitch();
  }

  private SwerveModuleState[] getModuleStates() {
    return modules.stream().map(ModuleIO::getState).toArray(SwerveModuleState[]::new);
  }

  private SwerveModulePosition[] getModulePositions() {
    return modules.stream().map(ModuleIO::getPosition).toArray(SwerveModulePosition[]::new);
  }

  /** Updates pose estimation based on provided {@link EstimatedRobotPose} */
  public void updateEstimates(EstimatedRobotPose... poses) {
    for (int i = 0; i < poses.length; i++) {
      odometry.addVisionMeasurement(poses[i].estimatedPose.toPose2d(), poses[i].timestampSeconds);
      field2d.getObject("Cam-" + i + " Est Pose").setPose(poses[i].estimatedPose.toPose2d());
    }
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getModulePositions());

    field2d.setRobotPose(getPose());

    for (int i = 0; i < modules2d.length; i++) {
      var module = modules.get(i);
      var transform = new Transform2d(MODULE_OFFSET[i], module.getPosition().angle);
      modules2d[i].setPose(getPose().transformBy(transform));
    }
  }

  @Override
  public void simulationPeriodic() {
    imu.getSimCollection()
        .addHeading(
            Units.radiansToDegrees(
                    kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond)
                * Constants.PERIOD);
  }

  /** Stops drivetrain */
  public CommandBase stop() {
    return runOnce(() -> drive(new ChassisSpeeds()));
  }

  /** Sets the drivetrain to an "X" configuration, preventing movement */
  public CommandBase lock() {
    var front = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    var back = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    return run(() -> setModuleStates(new SwerveModuleState[] {front, back, back, front}));
  }

  /** Sets a new speed multiplier for the robot, this affects max cartesian and angular speeds */
  public CommandBase setSpeedMultiplier(double multiplier) {
    return runOnce(() -> speedMultiplier = multiplier);
  }

  /**
   * Follows a simple {@link PathPlannerTrajectory} on the field.
   *
   * <p>For safer use, see {@link this#followPath(PathPlannerTrajectory, boolean)}.
   *
   * @param trajectory The trajectory to follow.
   * @return The command that follows the trajectory.
   */
  public CommandBase follow(PathPlannerTrajectory trajectory) {
    return new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            kinematics,
            new PIDController(TRANSLATION.p(), TRANSLATION.i(), TRANSLATION.d()),
            new PIDController(TRANSLATION.p(), TRANSLATION.i(), TRANSLATION.d()),
            new PIDController(ROTATION.p(), ROTATION.i(), ROTATION.d()),
            this::setModuleStates,
            false)
        .andThen(stop());
  }

  /**
   * Follows the specified trajectory using {@link this#follow(PathPlannerTrajectory)} and resets if
   * specified
   */
  public CommandBase followPath(PathPlannerTrajectory path, boolean resetOdometry) {
    var newPath = pathForAlliance(path, DriverStation.getAlliance());
    return (resetOdometry ? pathOdometryReset(newPath) : Commands.none()).andThen(follow(newPath));
  }

  /** Resets odometry to first pose in path, using ppl to reflects if using alliance color */
  public CommandBase pathOdometryReset(PathPlannerTrajectory trajectory) {
    var initialState = trajectory.getInitialState();
    Pose2d initialPose =
        new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
    return Commands.runOnce(() -> resetOdometry(initialPose), this);
  }

  /** Creates and follows trajectroy for swerve from startPose to desiredPose */
  public CommandBase driveToPose(Pose2d startPose, Pose2d desiredPose) {
    Rotation2d heading = desiredPose.minus(startPose).getTranslation().getAngle();
    PathPoint start = new PathPoint(startPose.getTranslation(), heading, startPose.getRotation());
    PathPoint goal =
        new PathPoint(desiredPose.getTranslation(), heading, desiredPose.getRotation());
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(CONSTRAINTS, start, goal);
    return follow(trajectory);
  }

  /** Creates and follows trajectory for swerve from current pose to desiredPose */
  public CommandBase driveToPose(Pose2d desiredPose) {
    return new DeferredCommand(() -> driveToPose(getPose(), desiredPose), this);
  }

  @Override
  public List<HardwareFault> getFaults() {
    var builder = new FaultBuilder();
    for (var module : modules) {
      builder.register(module.getFaults());
    }
    return builder.build();
  }

  public void close() throws Exception {
    frontLeft.close();
    frontRight.close();
    rearLeft.close();
    rearRight.close();
    imu.close();
  }
}
