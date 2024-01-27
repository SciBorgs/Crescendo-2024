package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Hashtable;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

import org.apache.commons.exec.DaemonExecutor;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.shooter.feeder.Feeder;
import org.sciborgs1155.robot.shooter.flywheel.Flywheel;
import org.sciborgs1155.robot.shooter.pivot.Pivot;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

public class Shooting implements Logged {

  @Log.NT private final Feeder feeder;
  @Log.NT private final Pivot pivot;
  @Log.NT private final Flywheel flywheel;

  private final Hashtable<Translation2d, ShooterState> shootingData = new Hashtable<>();

  public static record ShooterState(Rotation2d angle, double velocity) {}

  public Shooting(Flywheel flywheel, Pivot pivot, Feeder feeder) {
    this.flywheel = flywheel;
    this.pivot = pivot;
    this.feeder = feeder;
  }

  // shooting commands
  public Command shootStoredNote(DoubleSupplier desiredVelocity) {
    return Commands.parallel(
        flywheel.runFlywheel(() -> desiredVelocity.getAsDouble()),
        Commands.waitUntil(flywheel::atSetpoint).andThen(feeder.runFeeder(Volts.of(1))));
  }

  public Command pivotThenShoot(Supplier<Rotation2d> goalAngle, DoubleSupplier desiredVelocity) {
    return pivot
        .runPivot(goalAngle)
        .alongWith(Commands.waitUntil(pivot::atSetpoint).andThen(shootStoredNote(desiredVelocity)));
  }

  public boolean inShootingRange(Translation2d position) {
    return true;
  }

  public ShooterState getDesiredState(Translation2d position) {
    double x0 = Math.floor(position.getX() / DATA_INTERVAL) * DATA_INTERVAL;
    double x1 = Math.ceil(position.getX() / DATA_INTERVAL) * DATA_INTERVAL;
    double y0 = Math.floor(position.getY() / DATA_INTERVAL) * DATA_INTERVAL;
    double y1 = Math.ceil(position.getY() / DATA_INTERVAL) * DATA_INTERVAL;

    Translation2d point1 = new Translation2d(x0, y0);
    Translation2d point2 = new Translation2d(x0, y1);
    Translation2d point3 = new Translation2d(x1, y0);
    Translation2d point4 = new Translation2d(x1, y1);

    ShooterState state1 = shootingData.get(point1);
    ShooterState state2 = shootingData.get(point2);
    ShooterState state3 = shootingData.get(point3);
    ShooterState state4 = shootingData.get(point4);
    
    double distance1 = Math.pow(point1.getX() - position.getX(), 2) + Math.pow(point1.getY() - position.getY(), 2);
    double distance2 = Math.pow(point2.getX() - position.getX(), 2) + Math.pow(point2.getY() - position.getY(), 2);
    double distance3 = Math.pow(point3.getX() - position.getX(), 2) + Math.pow(point3.getY() - position.getY(), 2);
    double distance4 = Math.pow(point4.getX() - position.getX(), 2) + Math.pow(point4.getY() - position.getY(), 2);
    double distanceSum = distance1 + distance2 + distance3 + distance4;

    double factor1 = 1/distanceSum * distance1;
    double factor2 = 1/distanceSum * distance2;
    double factor3 = 1/distanceSum * distance3;
    double factor4 = 1/distanceSum * distance4;

    Rotation2d angle = new Rotation2d(Radians.of(factor1 * state1.angle().getRadians() + factor2 * state2.angle().getRadians() + factor3 * state3.angle().getRadians() + factor4 * state4.angle().getRadians()))
  }
}
