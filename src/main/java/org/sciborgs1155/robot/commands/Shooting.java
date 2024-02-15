package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.feeder.FeederConstants.FEEDER_VELOCITY;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Cache;
import org.sciborgs1155.robot.Cache.NoteTrajectory;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting {
  private final Shooter shooter;
  private final Pivot pivot;
  private final Feeder feeder;
  private final Drive drive;

  /** desired initial velocity of note, corresponds to pivot angle and shooter speed */
  public static record ShootingState(Rotation2d angle, double speed) {}

  public Shooting(Shooter shooter, Pivot pivot, Feeder feeder, Drive drive) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.feeder = feeder;
    this.drive = drive;
  }

  /**
   * Runs the shooter before feeding it the note.
   *
   * @param desiredVelocity The velocity to shoot at.
   * @return The command to shoot at the desired velocity.
   */
  public Command shoot(DoubleSupplier desiredVelocity) {
    return shooter
        .runShooter(desiredVelocity)
        .alongWith(
            Commands.waitUntil(shooter::atSetpoint)
                .andThen(feeder.runFeeder(FEEDER_VELOCITY.in(MetersPerSecond))));
  }

  /**
   * Runs the pivot to an angle & runs the shooter before feeding it the note.
   *
   * @param goalAngle The desired angle of the pivot.
   * @param shooterVelocity The desired velocity of the shooter.
   * @return The command to run the pivot to its desired angle and then shoot.
   */
  public Command pivotThenShoot(Supplier<Rotation2d> goalAngle, DoubleSupplier shooterVelocity) {
    return pivot
        .runPivot(goalAngle)
        .alongWith(shooter.runShooter(shooterVelocity))
        .alongWith(
            Commands.waitUntil(() -> pivot.atGoal() && shooter.atSetpoint())
                .andThen(feeder.runFeeder(FEEDER_VELOCITY.in(MetersPerSecond))));
  }

  public Command fullShooting(InputStream vx, InputStream vy, Translation2d pos, Vector<N2> vel) {
    return fullShooting(vx, vy, () -> Cache.getTrajectory(pos, vel));
  }
  
  public Command fullShooting(
      InputStream vx,
      InputStream vy,
      Supplier<NoteTrajectory> traj) {
    return shooter
        .runShooter(() -> traj.get().speed())
        .alongWith(pivot.runPivot(() -> traj.get().pivotAngle()))
        .alongWith(drive.drive(vx, vy, () -> traj.get().heading()))
        .alongWith(
            Commands.waitUntil(shooter::atSetpoint)
                .andThen(feeder.runFeeder(FEEDER_VELOCITY.in(MetersPerSecond))));
  }
}