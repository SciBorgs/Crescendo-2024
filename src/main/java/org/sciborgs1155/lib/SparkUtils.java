package org.sciborgs1155.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import java.util.Set;
import java.util.function.Function;
import org.sciborgs1155.lib.FaultLogger.FaultType;

/** Utility class for configuration of Spark motor controllers */
public class SparkUtils {

  public static final int FRAME_STRATEGY_DISABLED = 65535;
  public static final int FRAME_STRATEGY_SLOW = 400;
  public static final int FRAME_STRATEGY_MEDIUM = 100;
  public static final int FRAME_STRATEGY_FAST = 20;
  public static final int FRAME_STRATEGY_VERY_FAST = 10;

  public static final Angle ANGLE_UNIT = Units.Rotations;
  public static final Time TIME_UNIT = Units.Minutes;
  public static final int THROUGHBORE_CPR = 8192;

  public static final int MAX_ATTEMPTS = 5;

  public static String name(CANSparkBase spark) {
    return "Spark [" + spark.getDeviceId() + "]";
  }

  public static void configure(CANSparkBase spark, Function<CANSparkBase, REVLibError> config) {
    configure(spark, config, 2);
  }

  private static void configure(
      CANSparkBase spark, Function<CANSparkBase, REVLibError> config, int attempt) {
    if (attempt >= MAX_ATTEMPTS) {
      FaultLogger.report(name(spark), "FAILED TO SET PARAMETER", FaultType.ERROR);
      return;
    }
    REVLibError error = config.apply(spark);
    if (error != REVLibError.kOk) {
      configure(spark, config, attempt + 1);
    }
  }

  /** Represents a type of sensor that can be plugged into the spark */
  public static enum Sensor {
    INTEGRATED,
    ANALOG,
    ALTERNATE,
    ABSOLUTE;
  }

  /** Represents a type of data that can be sent from the spark */
  public static enum Data {
    POSITION,
    VELOCITY,
    CURRENT,
    TEMPERATURE,
    INPUT_VOLTAGE,
    APPLIED_OUTPUT;
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
    int status0 = FRAME_STRATEGY_MEDIUM; // output, faults
    int status1 = FRAME_STRATEGY_SLOW;
    // integrated velocity, temperature, input voltage, current | default 20
    int status2 = FRAME_STRATEGY_SLOW; // integrated position | default 20
    int status3 = FRAME_STRATEGY_DISABLED; // analog encoder | default 50
    int status4 = FRAME_STRATEGY_DISABLED; // alternate quadrature encoder | default 20
    int status5 = FRAME_STRATEGY_DISABLED; // duty cycle position | default 200
    int status6 = FRAME_STRATEGY_DISABLED; // duty cycle velocity | default 200
    int status7 = FRAME_STRATEGY_DISABLED;
    // status frame 7 is cursed, the only mention i found of it in rev's docs is at
    // https://docs.revrobotics.com/brushless/spark-flex/revlib/spark-flex-firmware-changelog#breaking-changes
    // if it's only IAccum, there's literally no reason to enable the frame

    if (withFollower || data.contains(Data.APPLIED_OUTPUT)) {
      status0 = FRAME_STRATEGY_VERY_FAST;
    }

    if (sensors.contains(Sensor.INTEGRATED) && data.contains(Data.VELOCITY)
        || data.contains(Data.INPUT_VOLTAGE)
        || data.contains(Data.CURRENT)
        || data.contains(Data.TEMPERATURE)) {
      status1 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.INTEGRATED) && data.contains(Data.POSITION)) {
      status2 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.ANALOG)
        && (data.contains(Data.VELOCITY) || data.contains(Data.POSITION))) {
      status3 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.ALTERNATE)
        && (data.contains(Data.VELOCITY) || data.contains(Data.POSITION))) {
      status4 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.ABSOLUTE)) {
      if (data.contains(Data.POSITION)) {
        status5 = FRAME_STRATEGY_FAST;
      }
      if (data.contains(Data.VELOCITY)) {
        status6 = FRAME_STRATEGY_FAST;
      }
    }

    setFrame(PeriodicFrame.kStatus0, status0, 0, spark);
    setFrame(PeriodicFrame.kStatus1, status1, 1, spark);
    setFrame(PeriodicFrame.kStatus2, status2, 2, spark);
    setFrame(PeriodicFrame.kStatus3, status3, 3, spark);
    setFrame(PeriodicFrame.kStatus4, status4, 4, spark);
    setFrame(PeriodicFrame.kStatus5, status5, 5, spark);
    setFrame(PeriodicFrame.kStatus6, status6, 6, spark);
    setFrame(PeriodicFrame.kStatus7, status7, 7, spark);
  }

  private static void setFrame(PeriodicFrame frame, int period, int i, CANSparkBase spark) {
    // TODO fix effectively final errors
    configure(spark, s -> s.setPeriodicFramePeriod(frame, period));
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
