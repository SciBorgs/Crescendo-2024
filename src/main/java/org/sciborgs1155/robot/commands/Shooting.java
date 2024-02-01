package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.flywheel.Flywheel;
import org.sciborgs1155.robot.pivot.Pivot;

public class Shooting {

  private final Feeder feeder;
  private final Pivot pivot;
  private final Flywheel flywheel;

  /** desired initial velocity of note, corresponds to pivot angle and flywheel speed */
  public static record ShooterState(Rotation2d angle, double speed) {}

  public Shooting(Flywheel flywheel, Pivot pivot, Feeder feeder) {
    this.flywheel = flywheel;
    this.pivot = pivot;
    this.feeder = feeder;
  }

  // shooting commands
  public Command shootStoredNote(DoubleSupplier desiredVelocity) {
    return Commands.parallel(
        flywheel.runFlywheel(() -> desiredVelocity.getAsDouble()),
        Commands.waitUntil(flywheel::atSetpoint).andThen(feeder.runFeeder(1)));
  }

  public Command pivotThenShoot(Supplier<Rotation2d> goalAngle, DoubleSupplier desiredVelocity) {
    return pivot
        .runPivot(goalAngle)
        .alongWith(Commands.waitUntil(pivot::atSetpoint).andThen(shootStoredNote(desiredVelocity)));
  }
}
