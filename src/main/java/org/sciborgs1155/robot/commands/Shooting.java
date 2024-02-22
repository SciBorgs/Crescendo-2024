package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting {
  private final Shooter shooter;
  private final Pivot pivot;
  private final Feeder feeder;

  /** desired initial velocity of note, corresponds to pivot angle and shooter speed */
  public static record ShootingState(Rotation2d angle, double speed) {}

  public Shooting(Shooter shooter, Pivot pivot, Feeder feeder) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.feeder = feeder;
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
                Commands.waitUntil(shooter::atSetpoint).andThen(feeder.forward()),
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
                pivot.runPivot(() -> goalAngle),
                shooter.runShooter(() -> shooterVelocity)));
  }
}
