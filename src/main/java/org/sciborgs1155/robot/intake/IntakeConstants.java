package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;

public final class IntakeConstants {
  public static final double INTAKE_SPEED = 0.8;
  public static final Measure<Current> CURRENT_LIMIT = Amps.of(50);
  public static final Measure<Time> RAMP_TIME = Milliseconds.of(50);
  public static final Measure<Time> DEBOUNCE_TIME = Seconds.of(0);
  public static final Measure<Time> INTAKE_FAST_PERIOD = Milliseconds.of(5);

  public static final Measure<Current> STALL_THRESHOLD = Amps.of(40);
}
