package org.sciborgs1155.robot.shooter.feeder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;

public class FeederConstants {
  public static final Measure<Current> CURRENT_LIMIT = Amps.of(1);

  public static final double GEARING = 1;

  public static final double kV = 0.5;
  public static final double kA = 1;
}
