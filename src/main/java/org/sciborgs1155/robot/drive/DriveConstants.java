package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Translation2d;

public final class DriveConstants {
  public static final double MAX_SPEED = 4.8; // m / s
  public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // rad / s
  public static final double MAX_ACCEL = 6.5; // m / s^2

  public static final double TRACK_WIDTH = 0.5715;
  // Distance between centers of right and left wheels on robot
  public static final double WHEEL_BASE = 0.5715;
  // Distance between front and back wheels on robot

  public static final Translation2d[] MODULE_OFFSET = {
    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // front left
    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // front right
    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // rear left
    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // rear right
  };

  public static final String[] MODULE_NAMES = {
    "front left module", "front right module", "rear left moduke", "rear right module"
  };

  // angular offsets of the modules, since we use absolute encoders
  // ignored (used as 0) in simulation because the simulated robot doesn't have offsets
  public static final double[] ANGULAR_OFFSETS = {
    -Math.PI / 2, // front left
    0, // front right
    Math.PI, // rear left
    Math.PI / 2 // rear right
  };

  public static final PIDConstants TRANSLATION = new PIDConstants(0.6, 0, 0);
  public static final PIDConstants ROTATION = new PIDConstants(0.4, 0, 0);

  public static final PathConstraints CONSTRAINTS =
      new PathConstraints(MAX_SPEED / 1.9, MAX_ACCEL / 1.4);

  public static final class SwerveModule {
    public static final class Driving {
      public static final double CIRCUMFERENCE = 2.0 * Math.PI * 0.0381;
      // Diameter of the wheel in meters (2 * π * R)

      public static final double GEARING = 1.0 / 45.0 / 22.0 * 15.0 * 14.0;

      public static final double CONVERSION = CIRCUMFERENCE * GEARING;

      public static final PIDConstants PID = new PIDConstants(0.11, 0, 0.06);
      public static final BasicFFConstants FF = new BasicFFConstants(0.3, 2.7, 0.25);
    }

    public static final class Turning {
      public static final double MOTOR_GEARING = 1.0 / 4.0 / 3.0;

      public static final double CONVERSION = 2.0 * Math.PI;

      public static final boolean ENCODER_INVERTED = true;

      public static final PIDConstants PID = new PIDConstants(2, 0, 0.1);
      // system constants only used in simulation
      public static final BasicFFConstants FF = new BasicFFConstants(0, 0.25, 0.015);
    }
  }
}
