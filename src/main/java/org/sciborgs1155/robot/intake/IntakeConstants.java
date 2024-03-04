package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;

public final class IntakeConstants {
  public static final double INTAKE_SPEED = 1;
  public static final Measure<Current> CURRENT_LIMIT = Amps.of(40);
  public static final Measure<Time> DEBOUNCE_TIME = Seconds.of(0);

  public static final Measure<Current> STALL_THRESHOLD =
      Amps.of(40); // TODO 40 is a MADE UP NUMBER someone fix dis
}
