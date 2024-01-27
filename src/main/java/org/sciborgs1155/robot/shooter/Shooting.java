package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Logged;
import org.sciborgs1155.robot.shooter.ShooterConstants.FlywheelConstants;
import org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants;
import org.sciborgs1155.robot.shooter.feeder.Feeder;
import org.sciborgs1155.robot.shooter.flywheel.Flywheel;
import org.sciborgs1155.robot.shooter.pivot.Pivot;

public class Shooting implements Logged {

  private final Feeder feeder;
  private final Pivot pivot;
  private final Flywheel flywheel;

  public Shooting(Flywheel flywheel, Pivot pivot, Feeder feeder) {
    this.flywheel = flywheel;
    this.pivot = pivot;
    this.feeder = feeder;
  }

  // shooting commands
  public Command shootStoredNote(DoubleSupplier desiredVelocity) {
    return Commands.parallel(
            flywheel.runFlywheel(() -> desiredVelocity.getAsDouble()),
            feeder.runFeeder(Volts.of(1)))
        .onlyIf(
            () ->
                flywheel.getVelocity()
                        <= desiredVelocity.getAsDouble()
                            + FlywheelConstants.VELOCITY_TOLERANCE.in(RadiansPerSecond)
                    && flywheel.getVelocity()
                        >= desiredVelocity.getAsDouble()
                            - FlywheelConstants.VELOCITY_TOLERANCE.in(RadiansPerSecond));
  }

  public Command pivotThenShoot(Supplier<Rotation2d> goalAngle, DoubleSupplier desiredVelocity) {
    return pivot
        .runPivot(goalAngle)
        .alongWith(
            shootStoredNote(desiredVelocity)
                .onlyIf(
                    () ->
                        pivot.getPosition().getRadians()
                                <= goalAngle.get().getRadians()
                                    + PivotConstants.POSITION_TOLERANCE.in(Radians)
                            && pivot.getPosition().getRadians()
                                >= goalAngle.get().getRadians()
                                    - PivotConstants.POSITION_TOLERANCE.in(Radians)));
  }
}
