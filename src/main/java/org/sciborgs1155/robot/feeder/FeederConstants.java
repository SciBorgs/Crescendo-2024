package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class FeederConstants {
  public static final Measure<Current> CURRENT_LIMIT = Amps.of(1);

  public static final double GEARING = 18.0/64.0;
  public static final Measure<Distance> RADIUS = Inches.of(0.63);
  
  public static final Measure<Distance> POSITION_CONVERSION = RADIUS.times(2*Math.PI).times(GEARING);
  public static final Measure<Velocity<Distance>> VELOCITY_CONVERSION = POSITION_CONVERSION.per(Minute);

  public static final double kV = 0.5;
  public static final double kA = 1;
}
