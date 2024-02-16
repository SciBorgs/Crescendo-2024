package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public static final Measure<Angle> POSITION_FACTOR = Rotations.of(THROUGBORE_GEARING);
  public static final Measure<Velocity<Angle>> VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

  // Offset from the center of the robot to the pivot's axis of rotation
  public static final Translation2d OFFSET = new Translation2d(Inches.of(10.465), Inches.of(25));

  public static final Measure<Mult<Mult<Distance, Distance>, Mass>> MOI =
      (Meters).mult(Meters).mult(Kilograms).of(0.17845);

  public static final Measure<Angle> POSITION_TOLERANCE = Degrees.of(1.0);

  public static final Measure<Mass> MASS = Pounds.of(14.973);
  public static final Measure<Distance> LENGTH = Inches.of(16);

  public static final Measure<Velocity<Angle>> MAX_VELOCITY = RadiansPerSecond.of(1.0);
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ACCEL = MAX_VELOCITY.per(Second);

  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-45.7);
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(63.3);

  public static final Rotation2d STARTING_ANGLE = MAX_ANGLE;

  public static final Rotation2d PRESET_SUBWOOFER_ANGLE = STARTING_ANGLE;
  public static final Rotation2d PRESET_AMP_ANGLE = MIN_ANGLE;

  public static final Measure<Current> CURRENT_LIMIT = Amps.of(30);

  public static final double kP = 2.5;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kS = 0.19945;
  public static final double kV = 0.0017622;
  public static final double kA = 0.00040123;
  public static final double kG = 0.043992;

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
