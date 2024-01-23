package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;

public class ShooterConstants {
  public static class FlywheelConstants {

    public static final double GEARING = 1;
    public static final Measure<Current> CURRENT_LIMIT = Amps.of(1);
    public static final double MOI = 1;

    public static final double RADIUS = 1;
    public static final Measure<Distance> CIRCUMFERENCE = Meters.of(2 * Math.PI * RADIUS);

    public static final Measure<Angle> POSITION_FACTOR =
        Rotations.of(GEARING).times(CIRCUMFERENCE.in(Meters));
    public static final Measure<Velocity<Angle>> VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 1;
    public static final double kV = 1;
    public static final double kA = 0;

    public static final double VELOCITY_TOLERANCE = .1;
  }

  public static class FeederConstants {
    public static final Measure<Current> CURRENT_LIMIT = Amps.of(1);

    public static final double GEARING = 1;

    public static final double kV = 1;
    public static final double kA = 1;
  }

  public static class PivotConstants {
    public static final double GEARING = 1;
    public static final double CONVERSION = 0;

    public static final double POSITION_OFFSET = 0;

    public static final double MASS = 6;
    public static final double LENGTH = 3;

    public static final double MAX_VELOCITY = 1;
    public static final double MAX_ACCEL = 1;

    public static final Measure<Angle> MAX_ANGLE = Radians.of(0);
    public static final Measure<Angle> MIN_ANGLE = Radians.of(0);

    public static final Measure<Angle> STARTING_ANGLE = Radians.of(0);

    public static final Measure<Current> CURRENT_LIMIT = Amps.of(6);

    public static final double kP = 5;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    public static final double POSITION_TOLERANCE = .1;

    public static final class ClimbConstants {
      public static final double kP = 1;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double MAX_VELOCITY = 1;
      public static final double MAX_ACCEL = 1;
      public static final double kS = 1;
      public static final double kG = 1;
      public static final double kV = 1;
    }
  }
}
