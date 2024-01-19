package org.sciborgs1155.robot.shooter;

public class ShooterConstants {
  public static class Flywheel {

    public static final double GEARING = -1;
    public static final int CURRENT_LIMIT = -1;
    public static final double MOI = -1;
    public static final double VELOCITY_CONVERSION = -1;

    public static final double kP = -1;
    public static final double kI = 0;
    public static final double kD = -1;

    public static final double kS = -1;
    public static final double kV = -1;
    public static final double kA = 0;
  }

  public static class Feeder {
    public static final int CURRENT_LIMIT = -1;

    public static final double GEARING = -1;

    public static final double kV = 1;
    public static final double kA = 1;
  }

  public static class Pivot {
    public static final double GEARING = 0;
    public static final double CONVERSION = 0;

    public static final double MASS = 0;
    public static final double LENGTH = 0;

    public static final double MAX_VELOCITY = 1;
    public static final double MAX_ACCEL = 1;

    public static final double MAX_ANGLE = 0;
    public static final double MIN_ANGLE = 0;

    public static final double STARTING_ANGLE = 0;

    public static final int CURRENT_LIMIT = 0;

    public static final double kP = -1;
    public static final double kI = 0;
    public static final double kD = -1;

    public static final double kS = -1;
    public static final double kV = -1;
    public static final double kA = 0;
  }
}
