package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class ShooterConstants {
  public static final Measure<Angle> PRESET_SUBWOOFER_ANGLE = Radians.of(1.22173);
  public static final Measure<Velocity<Angle>> PRESET_SUBWOOFER_VELOCITY = RadiansPerSecond.of(4);

  public static final double GEARING = 1;
  public static final double MOI = 1;

  public static final Measure<Distance> RADIUS = Meters.of(1);
  public static final Measure<Distance> CIRCUMFERENCE = Meters.of(2 * Math.PI * RADIUS.in(Meters));

  public static final Measure<Current> CURRENT_LIMIT = Amps.of(50);

  public static final Measure<Angle> POSITION_FACTOR =
      Rotations.of(GEARING).times(CIRCUMFERENCE.in(Meters));
  public static final Measure<Velocity<Angle>> VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

  public static final Measure<Velocity<Angle>> VELOCITY_TOLERANCE = RadiansPerSecond.of(1);

  public static final double kP = 2;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kS = 0;
  public static final double kV = 0.02;
  public static final double kA = 0;
}
