package org.sciborgs1155.robot;

import java.util.HashMap;
import java.util.Map;

public final class Ports {

  public static Map<Integer, String> idToName;

  static {
    idToName = new HashMap();
    // idToName.put(id, name);
    idToName.put(6, "shooter - top motor");
    idToName.put(9, "shooter - bottom motor");
    idToName.put(7, "pivot - top left motor");
    idToName.put(39, "pivot - right bottom motor");
    idToName.put(51, "pivot - left bottom motor");
    idToName.put(35, "pivot - right bottom motor");
    idToName.put(10, "feeder - motor");
    idToName.put(2, "drive - front left translational motor");
    idToName.put(4, "drive - rear left translational motor");
    idToName.put(3, "drive - front right translational motor");
    idToName.put(1, "drive - rear right translational motor");
    idToName.put(21, "drive - front left rotational motor");
    idToName.put(24, "drive - rear left rotational motor");
    idToName.put(34, "drive - front right rotational motor");
    idToName.put(20, "drive - rear right rotational motor");
    idToName.put(5, "intake - motor");
  }

  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Shooter {
    public static final int TOP_MOTOR = 6;
    public static final int BOTTOM_MOTOR = 9;
  }

  public static final class Pivot {
    public static final int SPARK_LEFT_TOP = 7;
    public static final int SPARK_RIGHT_TOP = 39;
    public static final int SPARK_LEFT_BOTTOM = 51;
    public static final int SPARK_RIGHT_BOTTOM = 35;
  }

  public static final class Feeder {
    public static final int FEEDER_SPARK = 10;
    public static final int BEAMBREAK = 0;
  }

  public static final class Drive {
    public static final int FRONT_LEFT_DRIVE = 2;
    public static final int REAR_LEFT_DRIVE = 4;
    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final int REAR_RIGHT_DRIVE = 1;

    public static final int FRONT_LEFT_TURNING = 21;
    public static final int REAR_LEFT_TURNING = 24;
    public static final int FRONT_RIGHT_TURNING = 34;
    public static final int REAR_RIGHT_TURNING = 20;
  }

  public static final class Intake {
    public static final int BEAMBREAK = 7;
    public static final int INTAKE_SPARK = 5;
  }

  public static final class Led {
    public static final int LED_PORT = 1; // led port
  }
}
