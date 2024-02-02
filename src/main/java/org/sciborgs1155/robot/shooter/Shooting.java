package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Hashtable;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.shooter.feeder.Feeder;
import org.sciborgs1155.robot.shooter.flywheel.Flywheel;
import org.sciborgs1155.robot.shooter.pivot.Pivot;

public class Shooting extends SubsystemBase implements Logged {
  
  @Log.NT private final Feeder feeder;
  @Log.NT private final Pivot pivot;
  @Log.NT private final Flywheel flywheel;
  
  @Log.NT public boolean shootability;

  private final Hashtable<Translation2d, ShooterState> shootingData;
  
  /** desired initial velocity of note, corresponds to pivot angle and flywheel speed */
  public static record ShooterState(Rotation2d angle, double speed) {}
  
  public Shooting(Flywheel flywheel, Pivot pivot, Feeder feeder) {
    this(flywheel, pivot, feeder, new Hashtable<Translation2d, ShooterState>());
  }

  public Shooting(
      Flywheel flywheel,
      Pivot pivot,
      Feeder feeder,
      Hashtable<Translation2d, ShooterState> shootingData) {
    this.shootingData = shootingData;
    this.flywheel = flywheel;
    this.pivot = pivot;
    this.feeder = feeder;
    shootability = true;
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

  private static double interpolate(double a, double b, double dist) {
    assert 0 <= dist && dist <= 1;
    return a * dist + b * (1 - dist);
  }

  private static ShooterState interpolateStates(ShooterState a, ShooterState b, double dist) {
    assert 0 <= dist && dist <= 1;
    return new ShooterState(
        Rotation2d.fromRadians(interpolate(a.angle().getRadians(), b.angle().getRadians(), dist)),
        interpolate(a.speed(), b.speed(), dist));
  }

  /** uses bilinear interpolation ({@link https://en.wikipedia.org/wiki/Bilinear_interpolation}) */
  public ShooterState desiredState(Translation2d position) throws Exception {
    double intervalMeters = DATA_INTERVAL.in(Meters);

    double x0 = Math.floor(position.getX() / intervalMeters) * intervalMeters;
    double x1 = Math.ceil(position.getX() / intervalMeters) * intervalMeters;
    double y0 = Math.floor(position.getY() / intervalMeters) * intervalMeters;
    double y1 = Math.ceil(position.getY() / intervalMeters) * intervalMeters;

    double x_dist = (position.getX() - x0) / intervalMeters;
    double y_dist = (position.getY() - y0) / intervalMeters;

    try {
      return interpolateStates(
          interpolateStates(
              shootingData.get(new Translation2d(x0, y0)),
              shootingData.get(new Translation2d(x1, y0)),
              x_dist),
          interpolateStates(
              shootingData.get(new Translation2d(x0, y1)),
              shootingData.get(new Translation2d(x1, y1)),
              x_dist),
          y_dist);
    } catch (Exception e) {
      throw (new Exception("cannot shoot from this position!"));
    }
  }

  public boolean canShoot() {
    // please code real code here
    return shootability;
  }
  public boolean cantShoot(){
    return !canShoot();
  }
}
