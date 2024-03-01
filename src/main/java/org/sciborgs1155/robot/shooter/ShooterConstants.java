package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class ShooterConstants {
  public static final double GEARING = 1;

  public static final Measure<Distance> RADIUS = Inches.of(4);
  public static final Measure<Distance> CIRCUMFERENCE = RADIUS.times(2 * Math.PI);

  public static final Measure<Current> CURRENT_LIMIT = Amps.of(30);

  public static final Measure<Angle> POSITION_FACTOR = Rotations.one();
  public static final Measure<Velocity<Angle>> VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

  public static final Measure<Velocity<Angle>> VELOCITY_TOLERANCE = RadiansPerSecond.of(5);

  public static final double kP = 0.18;
  public static final double kI = 0;
  public static final double kD = 0.00002;

  public static final double kS = 0;
  public static final double kV = 0.026444;
  public static final double kA = 0.007565;
}
