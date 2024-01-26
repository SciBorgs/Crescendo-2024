package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.shooter.ShooterConstants.FlywheelConstants;
import org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants;
import org.sciborgs1155.robot.shooter.feeder.Feeder;
import org.sciborgs1155.robot.shooter.flywheel.Flywheel;
import org.sciborgs1155.robot.shooter.pivot.Pivot;

public class Shooter extends SubsystemBase implements Logged {

  private final Feeder feeder;
  private final Pivot pivot;
  private final Flywheel flywheel;

  public Shooter(Flywheel flywheel, Pivot pivot, Feeder feeder) {
    this.flywheel = flywheel;
    this.pivot = pivot;
    this.feeder = feeder;
  }

  // shooting commands
  public Command shootStoredNote(DoubleSupplier desiredVelocity) {
    return Commands.parallel(
        Commands.run(() -> flywheel.runFlywheel(() -> desiredVelocity.getAsDouble())),
        Commands.run(() -> feeder.runFeeder(Volts.of(1)))
            .onlyIf(
                () ->
                    flywheel.getVelocity()
                            <= desiredVelocity.getAsDouble()
                                + FlywheelConstants.VELOCITY_TOLERANCE.in(RadiansPerSecond)
                        && flywheel.getVelocity()
                            >= desiredVelocity.getAsDouble()
                                - FlywheelConstants.VELOCITY_TOLERANCE.in(RadiansPerSecond)));
  }

  public Command pivotThenShoot(
      Supplier<Measure<Angle>> goalAngle, DoubleSupplier desiredVelocity) {
    return pivot
        .runPivot(goalAngle)
        .alongWith(
            shootStoredNote(desiredVelocity)
                .onlyIf(
                    () ->
                        pivot.getPosition()
                                <= goalAngle.get().in(Radians)
                                    + PivotConstants.POSITION_TOLERANCE.in(Radians)
                            && pivot.getPosition()
                                >= goalAngle.get().in(Radians)
                                    - PivotConstants.POSITION_TOLERANCE.in(Radians)));
  }

  public Command runPivot(Supplier<Measure<Angle>> goalAngle) {
    return pivot.runPivot(goalAngle);
  }

  public Command climb(Supplier<Measure<Angle>> goalAngle) {
    return pivot.climb(goalAngle);
  }

  // ProfilePID doesn't log this stuff
  @Log.NT
  public double getPivotSetpointRadians() {
    return pivot.getSetpointRadians().in(Radians);
  }
}
