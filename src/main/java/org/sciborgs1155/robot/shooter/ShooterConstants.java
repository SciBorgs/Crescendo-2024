package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;

public class ShooterConstants {
  public static final Measure<Distance> DATA_INTERVAL = Meters.of(2);

  public static class FlywheelConstants {
    public static final Measure<Angle> PRESET_SUBWOOFER_ANGLE = Radians.of(1.22173);
    public static final Measure<Velocity<Angle>> PRESET_SUBWOOFER_VELOCITY = RadiansPerSecond.of(4);

    public static final double GEARING = 1;
    public static final double MOI = 1;

    public static final Measure<Distance> RADIUS = Meters.of(1);
    public static final Measure<Distance> CIRCUMFERENCE =
        Meters.of(2 * Math.PI * RADIUS.in(Meters));

    public static final Measure<Current> CURRENT_LIMIT = Amps.of(10);

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

  public static class FeederConstants {
    public static final Measure<Current> CURRENT_LIMIT = Amps.of(1);

    public static final double GEARING = 1;

    public static final double kV = 0.5;
    public static final double kA = 1;
  }

  public static class PivotConstants {
    public static final double GEARING = 1;

    public static final Measure<Distance> RADIUS = Meters.of(1);
    public static final Measure<Distance> CIRCUMFERENCE =
        Meters.of(2 * Math.PI * RADIUS.in(Meters));

    public static final Measure<Angle> POSITION_FACTOR =
        Rotations.of(GEARING).times(CIRCUMFERENCE.in(Meters));

    public static final Measure<Angle> POSITION_OFFSET = Radians.of(0);
    public static final Measure<Angle> POSITION_TOLERANCE = Radians.of(0.1);

    public static final Measure<Mass> MASS = Kilograms.of(1);
    public static final Measure<Distance> LENGTH = Meters.of(3);

    public static final Measure<Velocity<Angle>> MAX_VELOCITY = RadiansPerSecond.of(0.5);
    public static final Measure<Velocity<Velocity<Angle>>> MAX_ACCEL = MAX_VELOCITY.per(Second);

    public static final Measure<Angle> MAX_ANGLE = Radians.of(Math.PI / 2);
    public static final Measure<Angle> MIN_ANGLE = Radians.of(0);

    public static final Measure<Angle> STARTING_ANGLE = Radians.of(0);

    public static final Measure<Current> CURRENT_LIMIT = Amps.of(6);

    public static final double kP = 70;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 5;
    public static final double kV = 10;
    public static final double kG = 10;

    public static final class ClimbConstants {
      public static final Measure<Velocity<Angle>> MAX_VELOCITY = RadiansPerSecond.of(0.2);
      public static final Measure<Velocity<Velocity<Angle>>> MAX_ACCEL = MAX_VELOCITY.per(Second);

      public static final double kP = 100;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double kS = 0;
      public static final double kG = 36;
      public static final double kV = 0;
    }
  }
}
