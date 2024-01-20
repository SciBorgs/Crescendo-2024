package org.sciborgs1155.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public final class FaultLogger {
  /** Adds an alert widget to SmartDashboard;. */
  public static void setupLogging() {
    SmartDashboard.putData(
        "Faults",
        builder -> {
          builder.setSmartDashboardType("Alerts");
          builder.addStringArrayProperty("errors", () -> getStrings(FaultType.ERROR), null);
          builder.addStringArrayProperty("warnings", () -> getStrings(FaultType.WARNING), null);
          builder.addStringArrayProperty("infos", () -> getStrings(FaultType.INFO), null);
        });
  }

  /** An individual fault, containing necessary information. */
  public static record Fault(String description, FaultType type, double timestamp) {
    public Fault(String description, FaultType type) {
      this(description, type, Timer.getFPGATimestamp());
    }

    @Override
    public boolean equals(Object other) {
      if (other instanceof Fault f) {
        return f.hashCode() == hashCode();
      }
      return false;
    }

    @Override
    public int hashCode() {
      return Objects.hash(description, type);
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
  private static final Set<Fault> faults = new HashSet<>();

  /** Polls registered fallibles. This method should be called periodically. */
  public static void update() {
    faultSuppliers.stream()
        .map(s -> s.get())
        .flatMap(Optional::stream)
        .collect(Collectors.toCollection(() -> faults));
  }

  /**
   * Returns a list of all current faults.
   *
   * @return A list of all current faults.
   */
  public static Set<Fault> getFaults() {
    return faults;
  }

  /**
   * Returns a list of current faults based on the provided FaultType.
   *
   * @param type The type of faults to return.
   * @return A list of faults of the specified FaultType.
   */
  public static List<Fault> getFaults(FaultType type) {
    return getFaults().stream().filter(a -> a.type() == type).toList();
  }

  /**
   * Returns a trigger for whether a failure has been reported.
   *
   * @param type The type of fault to filter for.
   * @return A trigger based on the presence of faults.
   */
  public static Trigger failing(FaultType type) {
    return new Trigger(() -> !getFaults(type).isEmpty());
  }

  /**
   * Configures a command to be ran whenever a failure is reported.
   *
   * @param command A function that creates a command from a set of faults.
   * @return The created trigger.
   */
  public static Trigger onFailing(Function<Set<Fault>, Command> command) {
    return failing(FaultType.ERROR).onTrue(command.apply(getFaults()));
  }

  /**
   * Returns an array of descriptions of all faults that match the specified type.
   *
   * @param type The type to filter for.
   * @return An array of description strings.
   */
  public static String[] getStrings(FaultType type) {
    return getFaults().stream()
        .filter(a -> a.type() == type)
        .map(Fault::description)
        .toArray(String[]::new);
  }

  /**
   * Registers a new fallible supplier.
   *
   * @param supplier A supplier of an optional fault.
   */
  public static void register(Supplier<Optional<Fault>> supplier) {
    faultSuppliers.add(supplier);
  }

  /**
   * Registers a new fallible supplier.
   *
   * @param condition Whether a failure is occuring.
   * @param description The failure's description.
   * @param type The type of failure.
   */
  public static void register(BooleanSupplier condition, String description, FaultType type) {
    faultSuppliers.add(
        () ->
            condition.getAsBoolean()
                ? Optional.of(new Fault(description, type))
                : Optional.empty());
  }

  /**
   * Registers fallible suppliers for a CAN-based Spark motor controller.
   *
   * @param spark The Spark Max or Spark Flex to manage.
   */
  public static void register(CANSparkBase spark) {
    int id = spark.getDeviceId();

    register(
        () -> {
          REVLibError err = spark.getLastError();
          return err == REVLibError.kOk
              ? Optional.empty()
              : Optional.of(
                  new Fault(
                      String.format("Spark [%d]: Error: %s", id, err.name()), FaultType.ERROR));
        });

    for (FaultID fault : FaultID.values()) {
      register(
          () -> spark.getFault(fault),
          String.format("Spark [%d]: Fault: %s", id, fault.name()),
          FaultType.WARNING);
    }
  }

  /**
   * Registers fallible suppliers for a duty cycle encoder.
   *
   * @param encoder The duty cycle encoder to manage.
   */
  public static void register(DutyCycleEncoder encoder) {
    register(
        () -> !encoder.isConnected(),
        String.format("DutyCycleEncoder [%d]: Disconnected"),
        FaultType.ERROR);
  }
}
