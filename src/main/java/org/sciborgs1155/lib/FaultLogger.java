package org.sciborgs1155.lib;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public final class FaultLogger {
  /** Adds an alert widget to SmartDashboard;. */
  public static void setupLogging() {
    SmartDashboard.putData(
        "Active Faults",
        builder -> {
          builder.setSmartDashboardType("Alerts");
          builder.addStringArrayProperty(
              "errors", () -> filteredStrings(activeFaults, FaultType.ERROR), null);
          builder.addStringArrayProperty(
              "warnings", () -> filteredStrings(activeFaults, FaultType.WARNING), null);
          builder.addStringArrayProperty(
              "infos", () -> filteredStrings(activeFaults, FaultType.INFO), null);
        });

    SmartDashboard.putData(
        "Total Faults",
        builder -> {
          builder.setSmartDashboardType("Alerts");
          builder.addStringArrayProperty(
              "errors", () -> filteredStrings(totalFaults, FaultType.ERROR), null);
          builder.addStringArrayProperty(
              "warnings", () -> filteredStrings(totalFaults, FaultType.WARNING), null);
          builder.addStringArrayProperty(
              "infos", () -> filteredStrings(totalFaults, FaultType.INFO), null);
        });
  }

  /** An individual fault, containing necessary information. */
  public static record Fault(String name, String description, FaultType type) {
    @Override
    public String toString() {
      return name + ": " + description;
    }
  }

  /**
   * The type of fault, used for detecting whether the fallible is in a failure state and displaying
   * to NetworkTables.
   */
  public static enum FaultType {
    INFO,
    WARNING,
    ERROR,
  }

  private static final List<Supplier<Optional<Fault>>> faultSuppliers = new ArrayList<>();
  private static final Set<Fault> activeFaults = new HashSet<>();
  private static final Set<Fault> totalFaults = new HashSet<>();

  /** Polls registered fallibles. This method should be called periodically. */
  public static void update() {
    activeFaults.clear();
    faultSuppliers.stream()
        .map(s -> s.get())
        .flatMap(Optional::stream)
        .forEach(FaultLogger::report);
    totalFaults.addAll(activeFaults);
  }

  /**
   * Returns the set of all current faults.
   *
   * @return The set of all current faults.
   */
  public static Set<Fault> activeFaults() {
    return activeFaults;
  }

  /**
   * Returns the set of all total faults.
   *
   * @return The set of all total faults.
   */
  public static Set<Fault> totalFaults() {
    return totalFaults;
  }

  /**
   * Returns a list of faults of the specified type from the set of faults.
   *
   * @param faults The set of faults to filter.
   * @param type The type of faults to find.
   * @return A list of faults of the specified FaultType.
   */
  public static List<Fault> filteredFaults(Set<Fault> faults, FaultType type) {
    return faults.stream().filter(a -> a.type() == type).toList();
  }

  /**
   * Returns an array of descriptions of all faults that match the specified type.
   *
   * @param type The type to filter for.
   * @return An array of description strings.
   */
  public static String[] filteredStrings(Set<Fault> faults, FaultType type) {
    return faults.stream()
        .filter(a -> a.type() == type)
        .map(Fault::toString)
        .toArray(String[]::new);
  }

  /**
   * Reports a fault.
   *
   * @param fault The fault to report.
   */
  public static void report(Fault fault) {
    activeFaults.add(fault);
    switch (fault.type) {
      case ERROR -> DriverStation.reportError(fault.toString(), false);
      case WARNING -> DriverStation.reportWarning(fault.toString(), false);
      case INFO -> System.out.println(fault.toString());
    }
  }

  /**
   * Reports a fault.
   *
   * @param name The name of the fault.
   * @param description The description of the fault.
   * @param type The type of the fault.
   */
  public static void report(String name, String description, FaultType type) {
    report(new Fault(name, description, type));
  }

  /**
   * Registers a new fault supplier.
   *
   * @param supplier A supplier of an optional fault.
   */
  public static void register(Supplier<Optional<Fault>> supplier) {
    faultSuppliers.add(supplier);
  }

  /**
   * Registers a new fault supplier.
   *
   * @param condition Whether a failure is occuring.
   * @param description The failure's description.
   * @param type The type of failure.
   */
  public static void register(
      BooleanSupplier condition, String name, String description, FaultType type) {
    faultSuppliers.add(
        () ->
            condition.getAsBoolean()
                ? Optional.of(new Fault(name, description, type))
                : Optional.empty());
  }

  /**
   * Registers fault suppliers for a CAN-based Spark motor controller.
   *
   * @param spark The Spark Max or Spark Flex to manage.
   */
  public static void register(CANSparkBase spark) {
    for (FaultID fault : FaultID.values()) {
      register(() -> spark.getFault(fault), SparkUtils.name(spark), fault.name(), FaultType.ERROR);
    }
  }

  /**
   * Registers fault suppliers for a duty cycle encoder.
   *
   * @param encoder The duty cycle encoder to manage.
   */
  public static void register(DutyCycleEncoder encoder) {
    register(
        () -> !encoder.isConnected(),
        "Duty Cycle Encoder [" + encoder.getSourceChannel() + "]",
        "disconnected",
        FaultType.ERROR);
  }

  /**
   * Registers fault suppliers for a NavX.
   *
   * @param ahrs The NavX to manage.
   */
  public static void register(AHRS ahrs) {
    register(() -> !ahrs.isConnected(), "NavX", "disconnected", FaultType.ERROR);
  }

  /**
   * Registers fault suppliers for a power distribution hub/panel.
   *
   * @param powerDistribution The power distribution to manage.
   */
  public static void register(PowerDistribution powerDistribution) {
    for (Field field : PowerDistribution.class.getFields()) {
      register(
          () ->
              IHateThisLanguage.handle(() -> field.getBoolean(powerDistribution))
                  .map(b -> new Fault("Power Distribution", field.getName(), FaultType.ERROR)));
    }
  }

  /**
   * Reports REVLibErrors from a spark.
   *
   * <p>This should be called immediately after any call to the spark.
   *
   * @param spark The spark to report REVLibErrors from.
   */
  public static void check(CANSparkBase spark) {
    REVLibError error = spark.getLastError();
    if (error != REVLibError.kOk) {
      report(SparkUtils.name(spark), error.name(), FaultType.ERROR);
    }
  }
}
