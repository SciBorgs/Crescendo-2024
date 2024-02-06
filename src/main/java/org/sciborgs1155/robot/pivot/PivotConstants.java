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
  public static final double GEARING = 12.0 / 64.0 * 20.0 / 70.0 * 36.0 / 56.0 * 16.0 / 54.0;

  public static void main(String[] args) {
    System.out.println(1 / GEARING);
  }

  public static final Measure<Distance> RADIUS = Meters.of(1);
  public static final Measure<Distance> CIRCUMFERENCE = Meters.of(2 * Math.PI * RADIUS.in(Meters));

  // Offset from the center of the robot
  public static final Translation2d OFFSET = new Translation2d(1, 0);

  // Height of the axis of rotation
  public static final Translation2d HEIGHT = new Translation2d(1, 0);

  public static final Measure<Mult<Mult<Distance, Distance>, Mass>> MOI =
      (Meters).mult(Meters).mult(Kilograms).of(0.17845);
  public static final Measure<Angle> POSITION_FACTOR =
      Rotations.of(GEARING).times(CIRCUMFERENCE.in(Meters));

  public static final Measure<Angle> POSITION_OFFSET = Radians.of(0);
  public static final Measure<Angle> POSITION_TOLERANCE = Radians.of(0.1);

  public static final Measure<Mass> MASS = Kilograms.of(1);
  public static final Measure<Distance> LENGTH = Meters.of(3);

  public static final Measure<Velocity<Angle>> MAX_VELOCITY = RadiansPerSecond.of(0.5);
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ACCEL = MAX_VELOCITY.per(Second);

  public static final Rotation2d MAX_ANGLE = Rotation2d.fromRadians(0);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(63.3);

  public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(63.3);

  public static final Measure<Current> CURRENT_LIMIT = Amps.of(50);

  // TODO DO NOT RUN THESE VALUES ON THE REAL PIVOT DO NOT DO THAT PLEASE DONT
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
