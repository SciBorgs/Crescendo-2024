package org.sciborgs1155.robot;

public final class Ports {
  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Shooter {
    public static final int TOP_MOTOR = -1;
    public static final int BOTTOM_MOTOR = -1;
  }

  public static final class Pivot {
    public static final int SPARK_LEFT_TOP = -1;
    public static final int SPARK_RIGHT_TOP = -1;
    public static final int SPARK_LEFT_BOTTOM = -1;
    public static final int SPARK_RIGHT_BOTTOM = -1;
    public static final int PIVOT_THROUGHBORE = -1;
  }

  public static final class Feeder {
    public static final int FEEDER_SPARK = -1;
    public static final int FRONT_BEAMBREAK = -1;
    public static final int END_BEAMBREAK = -1;
  }

  public static final class Drive {
    public static final int FRONT_LEFT_DRIVE = 11;
    public static final int REAR_LEFT_DRIVE = 10;
    public static final int FRONT_RIGHT_DRIVE = 12;
    public static final int REAR_RIGHT_DRIVE = 13;

    public static final int FRONT_LEFT_TURNING = 15;
    public static final int REAR_LEFT_TURNING = 14;
    public static final int FRONT_RIGHT_TURNING = 16;
    public static final int REAR_RIGHT_TURNING = 17;
  }

  public static final class Intake {
    public static final int BEAMBREAK = -1;
    public static final int INTAKE_SPARK = -1;
  }
}
