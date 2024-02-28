package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.pow;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.OFFSET;
import static org.sciborgs1155.robot.shooter.ShooterConstants.RADIUS;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_FLYWHEEL_SPEED;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting {

  private final Shooter shooter;
  private final Pivot pivot;
  private final Feeder feeder;
  private final Drive drive;

  public Shooting(Shooter shooter, Pivot pivot, Feeder feeder, Drive drive) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.feeder = feeder;
    this.drive = drive;
  }

  /**
   * Runs the shooter before feeding it the note.
   *
   * @param desiredVelocity The velocity in radians per second to shoot at.
   * @return The command to shoot at the desired velocity.
   */
  public Command shoot(Measure<Velocity<Angle>> desiredVelocity) {
    return shoot(() -> desiredVelocity, () -> true);
  }

  /**
   * Runs shooter to desired velocity, runs feeder once it reaches its velocity and shootCondition
   * is true.
   *
   * @param desiredVelocity Target velocity for the flywheel.
   * @param shootCondition Condition after which the feeder will run.
   */
  public Command shoot(
      Supplier<Measure<Velocity<Angle>>> desiredVelocity, BooleanSupplier shootCondition) {
    return (Commands.waitUntil(() -> shooter.atSetpoint() && shootCondition.getAsBoolean())
            .andThen(feeder.forward().withTimeout(0.15)))
        .deadlineWith(shooter.runShooter(() -> desiredVelocity.get().in(RadiansPerSecond)));
  }

  /**
   * Runs the pivot to an angle & runs the shooter before feeding it the note.
   *
   * @param targetAngle The desired angle of the pivot.
   * @param targetVelocity The desired velocity of the shooter.
   * @return The command to run the pivot to its desired angle and then shoot.
   */
  public Command pivotThenShoot(
      Measure<Angle> targetAngle, Measure<Velocity<Angle>> targetVelocity) {
    return shoot(() -> targetVelocity, pivot::atGoal).deadlineWith(pivot.runPivot(targetAngle));
  }

  /**
   * Shoots while moving.
   *
   * @param vx Supplier for x velocity of chassis
   * @param vy Supplier for y velocity of chassis
   */
  public Command shootWhileDriving(DoubleSupplier vx, DoubleSupplier vy) {
    return shoot(
            () -> flywheelSpeed(noteRobotRelativeVelocityVector()),
            () ->
                pivot.atGoal()
                    && drive.atHeadingGoal()
                    && drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond < 0.1)
        .deadlineWith(
            drive.drive(vx, vy, () -> heading(noteRobotRelativeVelocityVector())),
            pivot.runPivot(() -> pitch(noteRobotRelativeVelocityVector())));
  }

  /**
   * Converts heading, pivotAngle, and flywheel speed to the initial velocity of the note.
   *
   * @param heading
   * @param pitch
   * @param speed
   * @return Initial velocity vector of the note
   */
  public static Vector<N3> toVelocityVector(Rotation2d heading, double pitch, double speed) {
    double xyNorm = speed * Math.cos(pitch);
    double x = xyNorm * heading.getCos();
    double y = xyNorm * heading.getSin();
    double z = speed * Math.sin(pitch);
    return VecBuilder.fill(x, y, z);
  }

  /**
   * Calculates pitch from note initial velocity vector. If given a robot relative initial velocity
   * vector, the return value will also be the pivot angle.
   *
   * @param velocity Note initial velocity vector
   * @return Pitch/pivot angle
   */
  public static double pitch(Vector<N3> velocity) {
    return Math.atan(velocity.get(2) / VecBuilder.fill(velocity.get(0), velocity.get(1)).norm());
  }

  /**
   * Calculates heading from note initial velocity vector. If given a robot relative initial
   * velocity vector, the return value will be the target robot heading.
   *
   * @param velocity Note initial velocity vector
   * @return Heading
   */
  public static Rotation2d heading(Vector<N3> velocity) {
    return new Rotation2d(velocity.get(0), velocity.get(1));
  }

  /**
   * Calculates magnitude of initial velocity vector of note, in radians per second. If given a
   * robot relative initial velocity vector, the return value will be the target flywheel speed
   * (ish).
   *
   * @param velocity Note initial velocity vector
   * @return Flywheel speed (rads / s)
   */
  public static Measure<Velocity<Angle>> flywheelSpeed(Vector<N3> velocity) {
    // TODO account for lost energy! (with regression probably)
    return RadiansPerSecond.of(velocity.norm() / RADIUS.in(Meters));
  }

  public static Measure<Velocity<Distance>> noteSpeed(Measure<Velocity<Angle>> flywheelSpeed) {
    // TODO account for lost energy! (with regression probably)
    return MetersPerSecond.of(flywheelSpeed.in(RadiansPerSecond) * RADIUS.in(Meters));
  }
  
  /** Shoots while stationary at correct flywheel speed, pivot angle, and heading. */
  public Command stationaryTurretShooting() {
    Translation2d speaker = speaker().toTranslation2d();
    return new ProxyCommand(
        () -> {
          double targetPitch = pitch(drive.getPose(), noteSpeed(MAX_FLYWHEEL_SPEED)).in(Radians);
          return shoot(
                  () -> MAX_FLYWHEEL_SPEED,
                  () ->
                      pivot.atPosition(targetPitch)
                          && drive.isFacing(speaker)
                          && drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond < 0.1)
              .deadlineWith(
                  drive.driveFacingTarget(() -> 0, () -> 0, () -> speaker),
                  pivot.runPivot(() -> targetPitch))
              .andThen(Commands.print("DONE!!"));
        });
  }

  /**
   * Calculates heading needed to face the speaker.
   *
   * @param robotTranslation Translation2d of the robot
   */
  public static Rotation2d headingToSpeaker(Translation2d robotTranslation) {
    return speaker().toTranslation2d().minus(robotTranslation).getAngle();
  }

  /** Shoots while stationary at correct flywheel speed and pivot angle, doesn't auto-turret. */
  public Command stationaryShooting() {
    return new ProxyCommand(
        () ->
            pivotThenShoot(
                pitch(drive.getPose(), noteSpeed(MAX_FLYWHEEL_SPEED)), MAX_FLYWHEEL_SPEED));
  }

  /**
   * Calculates the required velocity vector of the note, relative to the robot. Accounts for the
   * velocity of the robot. This can be used to find flywheel speed, pivot angle and heading.
   *
   * @return Target initial velocity of note, relative to robot
   */
  public Vector<N3> noteRobotRelativeVelocityVector() {
    var heading = headingToSpeaker(drive.getPose().getTranslation());
    Vector<N3> stationaryVel =
        toVelocityVector(
            heading,
            pitch(drive.getPose(), noteSpeed(MAX_FLYWHEEL_SPEED)).in(Radians),
            noteSpeed(MAX_FLYWHEEL_SPEED).in(MetersPerSecond));
    var speeds = drive.getFieldRelativeChassisSpeeds();
    return stationaryVel.minus(
        VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0));
  }

  double G = 9.81; // I'm sorry (i'm tired) TODO

  public Measure<Angle> pitch(Pose2d robotPose, Measure<Velocity<Distance>> v) {
    double v_ = v.in(MetersPerSecond);
    Translation3d shooterPos = shooterPos(robotPose);
    double dist = speaker().toTranslation2d().getDistance(shooterPos.toTranslation2d());
    double h = speaker().getZ() - shooterPos.getZ();
    double denom = (G * pow(dist, 2));
    double rad =
        pow(dist, 2) * pow(v_, 4) - G * pow(dist, 2) * (G * pow(dist, 2) + 2 * h * pow(v_, 2));
    // TODO someone check me on this copying...
    return Radians.of(Math.atan((1 / (denom)) * (dist * pow(v_, 2) - Math.sqrt(rad))));
  }

  /** Calculates position of shooter from Pose2d of robot. */
  public static Translation3d shooterPos(Pose2d robotPose) {
    return new Pose3d(robotPose)
        .transformBy(new Transform3d(OFFSET, new Rotation3d()))
        .getTranslation();
  }

  /**
   * @return whether the robot can shoot from its current position at its current veloicty
   */
  public boolean canShoot() {
    Vector<N3> shot = noteRobotRelativeVelocityVector();
    double pitch = pitch(shot);
    return MIN_ANGLE.in(Radians) < pitch
        && pitch < MAX_ANGLE.in(Radians)
        && flywheelSpeed(shot).in(RadiansPerSecond) < DCMotor.getNeoVortex(1).freeSpeedRadPerSec;
  }
}
