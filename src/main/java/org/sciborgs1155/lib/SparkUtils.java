package org.sciborgs1155.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/** Utility class for configuration of Spark motor controllers */
public class SparkUtils {

  private static final List<Runnable> runnables = new ArrayList<>();

  public static void addChecker(Runnable runnable) {
    runnables.add(runnable);
  }

  public static List<Runnable> getRunnables() {
    return runnables;
  }

  // REV's docs have the size of a signed value of 65535ms for the max period
  // https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames
  // The actual max is half of this (32767ms)
  // https://www.chiefdelphi.com/t/revlibs-documented-maximum-status-frame-period-limit-is-wrong/458845, https://www.chiefdelphi.com/t/extreme-can-utilization-but-parameters-set-ok/456613/6
  public static final int FRAME_STRATEGY_DISABLED = 32767;
  public static final int FRAME_STRATEGY_SLOW = 400;
  public static final int FRAME_STRATEGY_MEDIUM = 100;
  public static final int FRAME_STRATEGY_FAST = 20;
  public static final int FRAME_STRATEGY_VERY_FAST = 10;

  public static final int THROUGHBORE_CPR = 8192;

  public static final int MAX_ATTEMPTS = 3;

  /**
   * Formats the name of a spark with its CAN ID.
   *
   * @param spark The spark to find the name of.
   * @return The name of a spark.
   */
  public static String name(CANSparkBase spark) {
    return "Spark [" + spark.getDeviceId() + "]";
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
  public static REVLibError configureFrameStrategy(
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
    // // status frame 7 is cursed, the only mention i found of it in rev's docs is at
    // //
    // https://docs.revrobotics.com/brushless/spark-flex/revlib/spark-flex-firmware-changelog#breaking-changes
    // // if it's only IAccum, there's literally no reason to enable the frame

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

    int[] frames = {status0, status1, status2, status3, status4, status5, status6, status7};
    REVLibError error = REVLibError.kOk;
    for (int i = 0; i < frames.length; i++) {
      REVLibError e = spark.setPeriodicFramePeriod(PeriodicFrame.fromId(i), frames[i]);
      if (e != REVLibError.kOk) {
        error = e;
      }
    }
    return error;
  }

  /**
   * Configures a follower spark to send nothing except output and faults. This means most data will
   * not be accessible.
   *
   * @param spark The follower spark.
   */
  public static REVLibError configureNothingFrameStrategy(CANSparkBase spark) {
    return configureFrameStrategy(spark, Set.of(), Set.of(), false);
  }

  /**
   * Wraps the value of a call into an optional depending on the spark's indicated last error.
   *
   * @param <T> The type of value.
   * @param spark The spark to check for errors.
   * @param value The value to wrap.
   * @return An optional that may contain the value.
   */
  public static <T> Optional<T> wrapCall(CANSparkBase spark, T value) {
    if (FaultLogger.check(spark)) {
      return Optional.of(value);
    }
    return Optional.empty();
  }
}
