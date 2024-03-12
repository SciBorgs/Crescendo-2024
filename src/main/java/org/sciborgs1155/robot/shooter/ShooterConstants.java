package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import org.sciborgs1155.robot.commands.Shooting;

public class ShooterConstants {
  public static final double GEARING = 1;

  public static final Measure<Distance> RADIUS = Inches.of(4);
  public static final Measure<Distance> CIRCUMFERENCE = RADIUS.times(2 * Math.PI);

  // Translation from center of robot to
  public static final Translation3d OFFSET =
      new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0));

  public static final Measure<Current> CURRENT_LIMIT = Amps.of(40);

  public static final Measure<Angle> POSITION_FACTOR = Rotations.one();
  public static final Measure<Velocity<Angle>> VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

  public static final Measure<Velocity<Angle>> VELOCITY_TOLERANCE = RadiansPerSecond.of(5);

  public static final Measure<Velocity<Angle>> IDLE_VELOCITY = RadiansPerSecond.of(200);
  public static final Measure<Velocity<Angle>> AMP_VELOCITY = RadiansPerSecond.of(150);

  public static final Measure<Velocity<Angle>> DEFAULT_VELOCITY = RadiansPerSecond.of(550);
  public static final Measure<Velocity<Distance>> DEFAULT_NOTE_VELOCITY =
      MetersPerSecond.of(Shooting.flywheelToNoteSpeed(DEFAULT_VELOCITY.in(RadiansPerSecond)));
  public static final Measure<Velocity<Angle>> MAX_VELOCITY = RadiansPerSecond.of(620);
  public static final Measure<Velocity<Distance>> MAX_NOTE_VELOCITY =
      MetersPerSecond.of(Shooting.flywheelToNoteSpeed(MAX_VELOCITY.in(RadiansPerSecond)));

  public static final double kP = 0.02;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0;
  public static final double kV = 0.016896;
  public static final double kA = 0.0031483;
}
