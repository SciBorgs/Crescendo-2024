package org.sciborgs1155.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import java.util.Set;

/** Utility class for configuration of Spark motor controllers */
public class SparkUtils {

  public static final int FRAME_STRATEGY_DISABLED = 65535;
  public static final int FRAME_STRATEGY_SLOW = 500;
  public static final int FRAME_STRATEGY_FAST = 20;

  public static final Angle ANGLE_UNIT = Units.Rotations;
  public static final Time TIME_UNIT = Units.Minutes;
  public static final int THROUGHBORE_CPR = 8192;

  /** Represents a type of sensor that can be plugged into the spark */
  public static enum Sensor {
    INTEGRATED,
    ANALOG,
    QUADRATURE,
    DUTY_CYCLE;
  }

  /** Represents a type of data that can be sent from the spark */
  public static enum Data {
    POSITION,
    VELOCITY,
    CURRENT,
    OUTPUT,
    INPUT;
  }

  /**
   * Configures CAN frames periods on a spark to send only specified data at high rates.
   *
   * @param spark The Spark MAX or Spark FLEX to configure.
   * @param data The data that the spark needs to send to the RIO.
   * @param sensors The sensors that provide data for the spark needs to send to the RIO.
   * @param withFollower Whether this spark has a following motor via {@link
   *     CANSparkBase#follow(CANSparkBase)}.
   * @see Sensor
   * @see Data
   * @see https://docs.revrobotics.com/brushless/spark-max/control-interfaces
   */
  public static void configureFrameStrategy(
      CANSparkBase spark, Set<Data> data, Set<Sensor> sensors, boolean withFollower) {
    int status0 = 10; // output, faults
    int status1 = FRAME_STRATEGY_SLOW; // velocity, temperature, input voltage, current
    int status2 = FRAME_STRATEGY_SLOW; // position
    int status3 = FRAME_STRATEGY_DISABLED; // analog encoder | default 50
    int status4 = FRAME_STRATEGY_DISABLED; // alternate quadrature encoder | default 20
    int status5 = FRAME_STRATEGY_DISABLED; // duty cycle position | default 200
    int status6 = FRAME_STRATEGY_DISABLED; // duty cycle velocity | default 200
    int status7 = FRAME_STRATEGY_DISABLED;

    if (!data.contains(Data.OUTPUT) && !withFollower) {
      status0 = FRAME_STRATEGY_SLOW;
    }

    if (data.contains(Data.VELOCITY) || data.contains(Data.INPUT) || data.contains(Data.CURRENT)) {
      status1 = FRAME_STRATEGY_FAST;
    }

    if (data.contains(Data.POSITION)) {
      status2 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.ANALOG)) {
      status3 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.QUADRATURE)) {
      status4 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.DUTY_CYCLE)) {
      if (data.contains(Data.POSITION)) {
        status5 = FRAME_STRATEGY_FAST;
      }
      if (data.contains(Data.VELOCITY)) {
        status6 = FRAME_STRATEGY_FAST;
      }
    }

    REVLibError[] errors = new REVLibError[8];
    errors[0] = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
    errors[1] = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
    errors[2] = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
    errors[3] = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, status3);
    errors[4] = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, status4);
    errors[5] = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, status5);
    errors[6] = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, status6);
    errors[7] = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus7, status7);
    // REVLibError e0 = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
    // REVLibError e1 = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
    // REVLibError e2 = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
    // REVLibError e3 = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, status3);
    // REVLibError e4 = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, status4);
    // REVLibError e5 = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, status5);
    // REVLibError e6 = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, status6);
    // REVLibError e7 = spark.setPeriodicFramePeriod(PeriodicFrame.kStatus7, status7);
    // REVLibError[] errors = {e0, e1, e2, e3, e4, e5, e6, e7};
    for (int i = 0; i < 8; i++) {
      if (errors[i] != REVLibError.kOk) {
        System.out.println("failed to set status frame " + i);
      }
    }
  }

  /**
   * Configures a follower spark to send nothing except output and faults. This means most data will
   * not be accessible.
   *
   * @param spark The follower spark.
   */
  public static void configureNothingFrameStrategy(CANSparkBase spark) {
    configureFrameStrategy(spark, Set.of(), Set.of(), false);
  }
}
