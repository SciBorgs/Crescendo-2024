package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting {

  private final Feeder feeder;
  private final Pivot pivot;
  private final Shooter shooter;

  /** desired initial velocity of note, corresponds to pivot angle and shooter speed */
  public static record ShootingState(Rotation2d angle, double speed) {}

  public Shooting(Shooter shooter, Pivot pivot, Feeder feeder) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.feeder = feeder;
  }

  // shooting commands
  public Command shoot(DoubleSupplier desiredVelocity) {
    return shooter
        .runShooter(desiredVelocity)
        .alongWith(Commands.waitUntil(shooter::atSetpoint).andThen(feeder.runFeeder(1)));
  }

  public Command pivotThenShoot(
      Supplier<Rotation2d> goalAngle,
      DoubleSupplier desiredVelocity,
      DoubleSupplier feederVelocity) {
    return pivot
        .runPivot(goalAngle)
        .alongWith(shooter.runShooter(desiredVelocity))
        .alongWith(
            Commands.waitUntil(pivot::atGoal)
                .andThen(feeder.runFeeder(feederVelocity.getAsDouble())));
  }
}
