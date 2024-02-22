package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;

public final class IntakeConstants {
  public static final double INTAKE_SPEED = 0.9;
  public static final Measure<Current> CURRENT_LIMIT = Amps.of(40);
}
