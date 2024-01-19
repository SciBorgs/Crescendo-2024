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
      public static final int PIVOT_THROUGHBORE = -1;
    }

    public static final class Feeder {
      public static final int FEEDER_SPARK = -1;
    }
  }
}
