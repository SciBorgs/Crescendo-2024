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

  public static record NoteTrajectory(Rotation2d heading, Rotation2d pivotAngle, double speed) {
    @Override
    public String toString() {
      return "{heading: "
          + heading.getRadians()
          + "; pivotAngle: "
          + pivotAngle.getRadians()
          + "; speed: "
          + speed
          + "}";
    }
  }

  /**
   * Runs the shooter before feeding it the note.
   *
   * @param desiredVelocity The velocity to shoot at.
   * @return The command to shoot at the desired velocity.
   */
  public Command shoot(DoubleSupplier desiredVelocity) {
    return shooter
        .setSetpoint(desiredVelocity)
        .andThen(
            Commands.deadline(
                Commands.waitUntil(shooter::atSetpoint)
                    .andThen(feeder.forward())
                    .andThen(Commands.waitSeconds(0.05)),
                shooter.runShooter(desiredVelocity)));
  }

  /**
   * Runs the pivot to an angle & runs the shooter before feeding it the note.
   *
   * @param goalAngle The desired angle of the pivot.
   * @param shooterVelocity The desired velocity of the shooter.
   * @return The command to run the pivot to its desired angle and then shoot.
   */
  public Command pivotThenShoot(double goalAngle, double shooterVelocity) {
    return pivot
        .setGoal(() -> goalAngle)
        .alongWith(shooter.setSetpoint(() -> shooterVelocity))
        .andThen(
            Commands.deadline(
                Commands.waitUntil(() -> pivot.atGoal() && shooter.atSetpoint())
                    .andThen(feeder.forward()),
                pivot.runPivot(goalAngle),
                shooter.runShooter(shooterVelocity)));
  }

  public Vector<N3> toVelocityVector(Rotation2d heading, double pivotAngle, double speed) {
    double z = speed * Math.sin(pivotAngle);
    double xyNorm = speed * Math.cos(pivotAngle);
    double x = xyNorm * heading.getCos();
    double y = xyNorm * heading.getSin();
    return VecBuilder.fill(x, y, z);
  }

  public Vector<N3> noteVelocityVector() {
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

  public static double pitch(Vector<N3> velocity) {
    return Math.atan(velocity.get(2) / VecBuilder.fill(velocity.get(0), velocity.get(1)).norm());
  }

  public static Rotation2d heading(Vector<N3> velocity) {
    return new Rotation2d(velocity.get(0), velocity.get(1));
  }

  public Command fullShooting(DoubleSupplier vx, DoubleSupplier vy) {
    return Commands.parallel(
        drive.drive(vx, vy, () -> heading(noteVelocityVector())),
        pivot.runPivot(() -> pitch(noteVelocityVector())),
        shooter.runShooter(() -> noteVelocityVector().norm()),
        Commands.waitUntil(
                () ->
                    pivot.atGoal()
                        && shooter.atSetpoint()
                        && drive.atHeadingGoal()
                        && drive.getChassisSpeeds().omegaRadiansPerSecond < 0.1)
            .andThen(feeder.forward()));
  }

  public Command stationaryShooting() {
    Supplier<Translation2d> speaker = () -> getSpeaker().toTranslation2d();
    Supplier<Translation2d> diff =
        () -> speaker.get().minus(shooterPos(drive.getPose()).toTranslation2d());
    return Commands.parallel(
            // drive.driveFacingTarget(() -> 0, () -> 0, () -> speaker.get()),
            Commands.runOnce(
                () -> System.out.println("SHOOTING VEL" + stationaryVelocity(diff.get()))),
            pivot.runPivot(() -> stationaryPitch(diff.get())),
            shooter.runShooter(() -> stationaryVelocity(diff.get())))
        .until(
            () -> pivot.atGoal() && shooter.atSetpoint()
            // && drive.isFacing(speaker.get())
            // && drive.getChassisSpeeds().omegaRadiansPerSecond < 0.1
            )
        .andThen(feeder.forward())
        .andThen(Commands.print("DONE!!"));
  }

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

  public static Translation3d shooterPos(Pose2d robotPose) {
    Translation2d shooterxy =
        robotPose
            .getTranslation()
            .plus(PIVOT_OFFSET.toTranslation2d().rotateBy(robotPose.getRotation()));
    return new Translation3d(shooterxy.getX(), shooterxy.getY(), PIVOT_OFFSET.getZ());
  }
}
