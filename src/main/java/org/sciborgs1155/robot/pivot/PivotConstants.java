package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Mult;
import edu.wpi.first.units.Velocity;

public class PivotConstants {
  public static final double MOTOR_GEARING = 12.0 / 64.0 * 20.0 / 70.0 * 36.0 / 56.0 * 16.0 / 54.0;
  public static final double THROUGBORE_GEARING = 16.0 / 54.0;

  public static final Measure<Distance> RADIUS = Meters.of(1);
  public static final Measure<Distance> CIRCUMFERENCE = Meters.of(2 * Math.PI * RADIUS.in(Meters));

  public static final Measure<Angle> POSITION_FACTOR = Rotations.of(MOTOR_GEARING);
  public static final Measure<Velocity<Angle>> VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

  /** Offset from the center of the robot to the pivot's axis of rotation */
  public static final Translation3d AXLE_FROM_CHASSIS =
      new Translation3d(Inches.of(-10.465), Inches.zero(), Inches.of(25));

  /** Offset from the pivot's axis of rotation to the shooter beambreak. */
  public static final Transform3d SHOOTER_FROM_AXLE =
      new Transform3d(
          new Translation3d(Inches.of(9.118), Inches.zero(), Inches.of(5.868)), new Rotation3d());

  public static final Measure<Mult<Mult<Distance, Distance>, Mass>> MOI =
      (Meters).mult(Meters).mult(Kilograms).of(0.17845);

  public static final Measure<Angle> POSITION_TOLERANCE = Degrees.of(1.0);

  public static final Measure<Mass> MASS = Kilograms.of(1);
  public static final Measure<Distance> LENGTH = Inches.of(16);

  public static final Measure<Velocity<Angle>> MAX_VELOCITY = RadiansPerSecond.of(8.0);
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ACCEL =
      RadiansPerSecond.per(Second).of(12.0);

  public static final Measure<Angle> STARTING_ANGLE = Degrees.of(63.3);

  public static final Measure<Angle> MIN_ANGLE = Degrees.of(-45.7);
  public static final Measure<Angle> MAX_ANGLE = STARTING_ANGLE.minus(Degrees.of(1));

  public static final Measure<Angle> PRESET_SUBWOOFER_ANGLE = STARTING_ANGLE;
  public static final Measure<Angle> PRESET_AMP_ANGLE = Radians.of(-0.55);
  public static final Measure<Angle> PRESET_PODIUM_ANGLE = Radians.of(0.5);

  public static final Measure<Current> CURRENT_LIMIT = Amps.of(60);

  public static final double kP = 8;
  public static final double kI = 0;
  public static final double kD = 0.5;

  public static final double kS = 0.14296;
  public static final double kV = 1.7305;
  public static final double kA = 0.12055;
  public static final double kG = 0.12055;

  public static final class ClimbConstants {
    public static final double kP = 64;
    public static final double kI = 0;
    public static final double kD = 0;

    // public static final double kS = 0;
    // public static final double kG = 36;
    // public static final double kV = 0;
  }
}
