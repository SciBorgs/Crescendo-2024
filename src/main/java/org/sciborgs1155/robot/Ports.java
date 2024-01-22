package org.sciborgs1155.robot;

public final class Ports {
  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Shooter {
    public static final class Flywheel {
      public static final int LEFT_MOTOR = -1;
      public static final int RIGHT_MOTOR = -1;
    }

    public static final class Pivot {
      public static final int PIVOT_SPARK_ONE = -1;
      public static final int PIVOT_SPARK_TWO = -1;
      public static final int PIVOT_SPARK_THREE = -1;
      public static final int PIVOT_SPARK_FOUR = -1;
      public static final int PIVOT_THROUGHBORE = -1;
    }

    public static final class Feeder {
      public static final int FEEDER_SPARK = -1;
    }
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
}
