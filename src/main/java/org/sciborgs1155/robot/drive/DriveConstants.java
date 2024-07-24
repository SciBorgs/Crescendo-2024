package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import java.util.List;

public final class DriveConstants {
  // Rate at which sensors update periodicially
  public static final Measure<Time> SENSOR_PERIOD = Seconds.of(0.001);

  // Distance between centers of right and left wheels on robot
  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0.5715);
  // Distance between front and back wheels on robot
  public static final Measure<Distance> WHEEL_BASE = Meters.of(0.5715);
  // Distance from the center to any wheel of the robot
  public static final Measure<Distance> RADIUS = TRACK_WIDTH.divide(2).times(Math.sqrt(2));
  // Robot width with bumpers
  public static final Measure<Distance> CHASSIS_WIDTH = Inches.of(32.645);

  // Maximum achievable translational and rotation velocities and accelerations of the robot.
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(5.74);
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCEL =
      MetersPerSecondPerSecond.of(16.0);
  public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED =
      RadiansPerSecond.of(MAX_SPEED.in(MetersPerSecond) / RADIUS.in(Meters));
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCEL =
      RadiansPerSecond.per(Second).of(MAX_ACCEL.in(MetersPerSecondPerSecond) / RADIUS.in(Meters));

  // Arbitrary max rotational velocity for the driver to effectively control the robot
  public static final Measure<Velocity<Angle>> TELEOP_ANGULAR_SPEED =
      Radians.per(Second).of(2 * Math.PI);

  public static final Translation2d[] MODULE_OFFSET = {
    new Translation2d(WHEEL_BASE.divide(2), TRACK_WIDTH.divide(2)), // front left
    new Translation2d(WHEEL_BASE.divide(2), TRACK_WIDTH.divide(-2)), // front right
    new Translation2d(WHEEL_BASE.divide(-2), TRACK_WIDTH.divide(2)), // rear left
    new Translation2d(WHEEL_BASE.divide(-2), TRACK_WIDTH.divide(-2)) // rear right
  };

  // angular offsets of the modules, since we use absolute encoders
  // ignored (used as 0) in simulation because the simulated robot doesn't have offsets
  public static final List<Rotation2d> ANGULAR_OFFSETS =
      List.of(
          Rotation2d.fromRadians(-Math.PI / 2), // front left
          Rotation2d.fromRadians(0), // front right
          Rotation2d.fromRadians(Math.PI), // rear left
          Rotation2d.fromRadians(Math.PI / 2) // rear right
          );

  public static final Rotation3d GYRO_OFFSET = new Rotation3d(0, 0, Math.PI);

  public static final class Translation {
    public static final double P = 3.0;
    public static final double I = 0.0;
    public static final double D = 0.05;

    public static final Measure<Distance> TOLERANCE = Centimeters.of(5);
  }

  public static final class Rotation {
    public static final double P = 4.5;
    public static final double I = 0.0;
    public static final double D = 0.05;

    public static final Measure<Angle> TOLERANCE = Degrees.of(3);
  }

  public static final class ModuleConstants {
    public static final double COUPLING_RATIO = 0;

    public static final class Driving {
      // Possible pinion configurations : 12T, 13T, or 14T.
      public static final int PINION_TEETH = 14;

      public static final Measure<Distance> CIRCUMFERENCE = Meters.of(2.0 * Math.PI * 0.0381);

      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
      // bevel pinion
      public static final double GEARING = 1.0 / 45.0 / 22.0 * 15.0 * 14.0;

      public static final Measure<Distance> POSITION_FACTOR = CIRCUMFERENCE.times(GEARING);
      public static final Measure<Velocity<Distance>> VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

      public static final Measure<Current> CURRENT_LIMIT = Amps.of(50);

      public static final class PID {
        public static final class SPARK {
          public static final double P = 1;
          public static final double I = 0.0;
          public static final double D = 0.0;
        }

        public static final class TALON {
          public static final double P = 3.2;
          public static final double I = 0.0;
          public static final double D = 0.0;
        }

        public static final class SIM {
          public static final double P = 3.2;
          public static final double I = 0.0;
          public static final double D = 0.0;
        }
      }

      public static final class FF {
        // s: 0.21474, 0.23963, 0.16188, 0.13714
        // v: 2.115, 2.0681, 2.1498, 2.0948
        // a linear: 0.17586, 0.13707, 0.23915, 0.26842
        // a rotation: 0.37587, 0.20079
        // 2 has 0.55 R^2
        public static final class SPARK {
          public static final double S = 0.23963;
          public static final double V = 2.0681;
          public static final double kA_linear = 0.205;
          public static final double kA_angular = 0.376;
        }

        public static final class TALON {
          public static final double S = -1; // TODO
          public static final double V = 2.0681;
          public static final double kA_linear = 0.205;
          public static final double kA_angular = 0.376;
        }
      }
    }

    static final class Turning {
      public static final double MOTOR_GEARING = 1.0 / 4.0 / 3.0;
      public static final double ENCODER_GEARING = 1;

      public static final Measure<Angle> POSITION_FACTOR = Rotations.of(ENCODER_GEARING);
      public static final Measure<Velocity<Angle>> VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

      public static final boolean ENCODER_INVERTED = true;

      public static final Measure<Current> CURRENT_LIMIT = Amps.of(20);

      public static final class PID {
        public static final class SPARK {
          public static final double P = 0.3; // FAKE DOES NOT WORK
          public static final double I = 0.0;
          public static final double D = 0.05;
        }

        public static final class TALON {
          public static final double P = 9;
          public static final double I = 0.0;
          public static final double D = 0.05;
        }

        public static final class SIM {
          public static final double P = 9;
          public static final double I = 0.0;
          public static final double D = 0.05;
        }
      }

      // system constants only used in simulation
      public static final class FF {
        public static final double S = 0.30817;
        public static final double V = 0.55;
        public static final double A = 0.03;
      }
    }
  }
}
