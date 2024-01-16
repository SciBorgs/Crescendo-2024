package org.sciborgs1155.robot.climber;

public class ClimberConstants {
  public static final double kP = 1;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;
  public static final double kG = 0.8;

  public static final double MAX_VELOCITY = 1;
  public static final double MAX_ACCELERATION = 1;

  public static final double DRUM_GEARS = 1;
  public static final double MOTOR_GEARS = 2;

  public static final double MINIMUM_CLIMBER_LENGTH = 1;
  public static final double MAXIMUM_CLIMBER_LENGTH = 5;
  public static final double HOOK_LENGTH = 1;
  public static final double CLIMBER_MASS = 1;
  public static final double RADIUS_METERS = 1;
  public static final double G = (MOTOR_GEARS - DRUM_GEARS) / MOTOR_GEARS * 100;

  public static final double DISTANCE_PER_PULSE = 1;
}
