package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class FeederConstants {
  public static final Measure<Current> CURRENT_LIMIT = Amps.of(50);
  public static final Measure<Velocity<Distance>> FEEDER_VELOCITY = MetersPerSecond.of(1);
  public static final double GEARING = 18.0 / 64.0;
  public static final Measure<Distance> RADIUS = Inches.of(0.63);

  public static final Measure<Distance> POSITION_CONVERSION =
      RADIUS.times(2 * Math.PI).times(GEARING);
  public static final Measure<Velocity<Distance>> VELOCITY_CONVERSION =
      POSITION_CONVERSION.per(Minute);

  public static final double kP = 0.6;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kS = 0;
  public static final double kV = 0.2;
  public static final double kA = 0.7;
}
