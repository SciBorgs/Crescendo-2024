package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.feeder.FeederConstants.FEEDER_VELOCITY;
import static org.sciborgs1155.robot.pivot.PivotConstants.PIVOT_OFFSET;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.InputStream;
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
                Commands.waitUntil(shooter::atSetpoint).andThen(feeder.eject()),
                shooter.runShooter(desiredVelocity)));
  }

  /**
   * Runs the pivot to an angle & runs the shooter before feeding it the note.
   *
   * @param goalAngle The desired angle of the pivot.
   * @param shooterVelocity The desired velocity of the shooter.
   * @return The command to run the pivot to its desired angle and then shoot.
   */
  public Command pivotThenShoot(DoubleSupplier goalAngle, DoubleSupplier shooterVelocity) {
    return pivot
        .setGoal(goalAngle)
        .alongWith(shooter.setSetpoint(shooterVelocity))
        .andThen(
            Commands.deadline(
                Commands.waitUntil(() -> pivot.atGoal() && shooter.atSetpoint())
                    .andThen(feeder.eject()),
                pivot.runPivot(goalAngle),
                shooter.runShooter(shooterVelocity)));
  }

  public Command fullShooting(
      InputStream vx,
      InputStream vy,
      DoubleSupplier flywheelVel,
      Supplier<Rotation2d> heading,
      Supplier<Rotation2d> pitch) {
    return Commands.parallel(
        shooter.runShooter(() -> flywheelVel.getAsDouble()),
        pivot.runPivot(() -> pitch.get()),
        drive.drive(vx, vy, () -> heading.get()),
        Commands.waitUntil(
                () ->
                    pivot.atGoal()
                        && shooter.atSetpoint()
                        && Math.abs(drive.getHeading().minus(heading.get()).getRadians()) < 0.01
                        && drive.getChassisSpeeds().omegaRadiansPerSecond < 0.01)
            .andThen(feeder.runFeeder(FEEDER_VELOCITY.in(MetersPerSecond))));
  }

  public Command stationaryShooting(double flywheelVel, Rotation2d heading, Rotation2d pitch) {
    return fullShooting(() -> 0, () -> 0, () -> flywheelVel, () -> heading, () -> pitch);
  }

  public Command stationaryShooting() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      var fromSpeaker = tranlationFromSpeaker(drive.getPose().getTranslation(), alliance.get());
      return stationaryShooting(
          stationaryVelocity(fromSpeaker),
          fromSpeaker.times(-1).getAngle(),
          stationaryPitch(fromSpeaker));
    }
    return Commands.none();
  }

  public static double stationaryVelocity(Translation2d translationFromSpeaker) {
    double x = translationFromSpeaker.getX();
    double y = translationFromSpeaker.getY();
    return 1.1331172768630184
        + 0.0337170229983295 * x
        + -0.07822480760293148 * y
        + -0.010386903450326593 * x * x
        + -0.00030007103195798433 * y * y
        + -0.0042478354516679185 * x * y;
  }

  public static Rotation2d stationaryPitch(Translation2d translationFromSpeaker) {
    double x = translationFromSpeaker.getX();
    double y = translationFromSpeaker.getY();
    return Rotation2d.fromRadians(
        1.1331172768630184
            + 0.0337170229983295 * x
            + -0.07822480760293148 * y
            + -0.010386903450326593 * x * x
            + -0.00030007103195798433 * y * y
            + -0.0042478354516679185 * x * y);
  }

  public static Translation2d tranlationFromSpeaker(Translation2d position, Alliance alliance) {
    return (switch (alliance) {
          case Blue -> BLUE_SPEAKER_POSE;
          case Red -> RED_SPEAKER_POSE;
        })
        .minus(position);
  }

  public static Translation3d shooterPos(Pose2d robotPose) {
    var robotTrans = robotPose.getTranslation();
    var robotHeading = robotPose.getRotation();
    var offset2d = new Translation2d(PIVOT_OFFSET.getX(), PIVOT_OFFSET.getY());
    var shooterxy = robotTrans.plus(offset2d.rotateBy(robotHeading));
    return new Translation3d(shooterxy.getX(), shooterxy.getY(), PIVOT_OFFSET.getZ());
  }
}
