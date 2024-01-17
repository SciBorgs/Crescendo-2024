package org.sciborgs1155.robot.shooter;

public class ShooterConstants {
  public static class Flywheel {
    public static final double GEARING = -1;

    public static class PID {
      public static final double kP = -1;
      public static final double kI = 0;
      public static final double kD = -1;
    }

    public static class FF {
      public static final double kS = -1;
      public static final double kV = -1;
      public static final double kA = 0;
    }

    public static final int CURRENT_LIMIT = -1;
    public static final double MOI = -1;
  }

  public static class Feeder {
    public static final double GEARING = -1;
  }

  public static class Pivot {
    public static class PID {
      public static final double kP = -1;
      public static final double kI = 0;
      public static final double kD = -1;
    }

    public static class FF {
      public static final double kS = -1;
      public static final double kV = -1;
      public static final double kA = 0;
    }
  }
}
