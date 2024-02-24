package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.PIVOT_OFFSET;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
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
  public Command shoot(double desiredVelocity) {
    return Commands.deadline(
        Commands.waitUntil(() -> shooter.atVelocity(desiredVelocity))
            .andThen(feeder.forward().withTimeout(0.05)),
        shooter.runShooter(() -> desiredVelocity));
  }

  /**
   * Runs the pivot to an angle & runs the shooter before feeding it the note.
   *
   * @param targetAngle The desired angle of the pivot.
   * @param targetVelocity The desired velocity of the shooter.
   * @return The command to run the pivot to its desired angle and then shoot.
   */
  public Command pivotThenShoot(double targetAngle, double targetVelocity) {
    return Commands.deadline(
        Commands.waitUntil(
                () -> pivot.atPosition(targetAngle) && shooter.atVelocity(targetVelocity))
            .andThen(feeder.forward().withTimeout(0.05)),
        pivot.runPivot(targetAngle),
        shooter.runShooter(targetVelocity));
  }

  /**
   * Shoots while moving.
   *
   * @param vx Supplier for x velocity of chassis
   * @param vy Supplier for y velocity of chassis
   */
  public Command fullShooting(DoubleSupplier vx, DoubleSupplier vy) {
    return Commands.parallel(
        drive.drive(vx, vy, () -> heading(noteRobotRelativeVelocityVector())),
        pivot.runPivot(() -> pitch(noteRobotRelativeVelocityVector())),
        shooter.runShooter(() -> noteRobotRelativeVelocityVector().norm()),
        Commands.waitUntil(
                () ->
                    pivot.atGoal()
                        && shooter.atSetpoint()
                        && drive.atHeadingGoal()
                        && drive.getChassisSpeeds().omegaRadiansPerSecond < 0.1)
            .andThen(feeder.forward()));
  }

  /**
   * Converts heading, pivotAngle, and flywheel speed to the initial velocity of the note.
   *
   * @param heading
   * @param pivotAngle
   * @param speed
   * @return Initial velocity vector of the note
   */
  public Vector<N3> toVelocityVector(Rotation2d heading, double pivotAngle, double speed) {
    double xyNorm = speed * Math.cos(pivotAngle);
    double x = xyNorm * heading.getCos();
    double y = xyNorm * heading.getSin();
    double z = speed * Math.sin(pivotAngle);
    return VecBuilder.fill(x, y, z);
  }

  /**
   * Calculates the required velocity vector of the note, relative to the robot. Accounts for the
   * velocity of the robot. This can be used to find flywheel speed, pivot angle and heading.
   *
   * @return Target initial velocity of note, relative to robot
   */
  public Vector<N3> noteRobotRelativeVelocityVector() {
    var speaker = getSpeaker().toTranslation2d();
    var fromSpeaker = shooterPos(drive.getPose()).toTranslation2d().minus(speaker);
    var stationaryVel =
        toVelocityVector(
            speaker.minus(drive.getPose().getTranslation()).getAngle(),
            stationaryPitch(fromSpeaker),
            stationaryVelocity(fromSpeaker));
    var speeds = drive.getChassisSpeeds();
    return stationaryVel.minus(
        VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0));
  }

  /**
   * Calculates pitch from note initial velocity vector. If given a robot relative initial velocity
   * vector, the return value will also be the pivot angle.
   *
   * @param Note initial velocity vector
   * @return Pitch/pivot angle
   */
  public static double pitch(Vector<N3> velocity) {
    return Math.atan(velocity.get(2) / VecBuilder.fill(velocity.get(0), velocity.get(1)).norm());
  }

  /**
   * Calculates heading from note initial velocity vector. If given a robot relative initial
   * velocity vector, the return value will be the target robot heading.
   *
   * @param Note initial velocity vector.
   * @return Heading
   */
  public static Rotation2d heading(Vector<N3> velocity) {
    return new Rotation2d(velocity.get(0), velocity.get(1));
  }

  /** Shoots while stationary at correct flywheel speed, pivot angle, and heading. */
  public Command stationaryTurretShooting() {
    Translation2d speaker = getSpeaker().toTranslation2d();
    Translation2d tranlationFromSpeaker =
        speaker.minus(shooterPos(drive.getPose()).toTranslation2d());
    return Commands.parallel(
            drive.driveFacingTarget(() -> 0, () -> 0, () -> speaker),
            pivot.runPivot(stationaryPitch(tranlationFromSpeaker)),
            shooter.runShooter(stationaryVelocity(tranlationFromSpeaker)))
        .until(
            () ->
                pivot.atGoal()
                    && shooter.atSetpoint()
                    && drive.isFacing(speaker)
                    && drive.getChassisSpeeds().omegaRadiansPerSecond < 0.1)
        .andThen(feeder.forward())
        .andThen(Commands.print("DONE!!"));
  }

  /** Shoots while stationary at correct flywheel speed and pivot angle, doesn't auto-turret. */
  public Command stationaryShooting() {
    Translation2d tranlationFromSpeaker =
        getSpeaker().minus(shooterPos(drive.getPose())).toTranslation2d();
    double targetVel = stationaryVelocity(tranlationFromSpeaker);
    return Commands.parallel(
            Commands.runOnce(() -> System.out.println("SHOOTING VEL" + targetVel)),
            pivot.runPivot(stationaryPitch(tranlationFromSpeaker)),
            shooter.runShooter(targetVel))
        .until(() -> pivot.atGoal() && shooter.atSetpoint())
        .andThen(feeder.forward())
        .withTimeout(1) // do we even need this? It can just stop when the button is released
        .andThen(Commands.print("DONE!!"));
  }

  /**
   * Uses regression from casadi model to calculate the target initial speed of the note, field
   * relative.
   *
   * @param translationFromSpeaker Tranlation of shooter from speaker (speaker - shooter)
   * @return Target speed of note.
   */
  public static double stationaryVelocity(Translation2d translationFromSpeaker) {
    double x = Math.abs(translationFromSpeaker.getX());
    double y = translationFromSpeaker.getY();
    return 5.5271852433873345
        + 0.5190955547104696 * x
        + 0.07707519916670269 * y
        + -0.0024133794816567633 * x * x
        + 0.01498409775890086 * y * y
        + -0.06372350453603381 * x * y;
  }

  /**
   * Uses regression from casadi model to calculate the target initial pitch of the note, field
   * relative.
   *
   * @param translationFromSpeaker Tranlation of shooter from speaker (speaker - shooter)
   * @return Target pitch of note.
   */
  public static double stationaryPitch(Translation2d translationFromSpeaker) {
    double x = Math.abs(translationFromSpeaker.getX());
    double y = translationFromSpeaker.getY();
    return 0.8362824373065703
        + -0.15055912579435599 * x
        + 0.13152228760076837 * y
        + -0.0056287323828513765 * x * x
        + -0.020130400003326433 * y * y
        + 0.014766007030961469 * x * y;
  }

  /** Calculates position of shooter from Pose2d of robot. */
  public static Translation3d shooterPos(Pose2d robotPose) {
    Translation2d shooterxy =
        robotPose
            .getTranslation()
            .plus(PIVOT_OFFSET.toTranslation2d().rotateBy(robotPose.getRotation()));
    return new Translation3d(shooterxy.getX(), shooterxy.getY(), PIVOT_OFFSET.getZ());
  }
}
