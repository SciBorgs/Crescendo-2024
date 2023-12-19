package org.sciborgs1155.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.REVLibError;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;

/** Functional interface to represent a hardware or software component that can fail. */
@FunctionalInterface
public interface Fallible extends Sendable {

  /** An individual fault, containing necessary information. */
  public static record Fault(String description, FaultType type, double timestamp) {
    public Fault(String description, FaultType type) {
      this(description, type, Timer.getFPGATimestamp());
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

  public static double DEFAULT_DEBOUNCE_TIME = 0.1;

  /**
   * Returns a list of all current faults.
   *
   * @return A list of all current faults.
   */
  public List<Fault> getFaults();

  /**
   * Returns whether the fallible is failing.
   *
   * @return Whether or not a fault with status {@code FaultType.ERROR} is present in {@link
   *     #getFaults()}.
   */
  public default boolean isFailing() {
    for (Fault fault : getFaults()) {
      if (fault.type() == FaultType.ERROR) {
        return true;
      }
    }
    return false;
  }

  /**
   * Returns a trigger for when the fallible is failing.
   *
   * @return A trigger based on {@link #isFailing()}.
   */
  public default Trigger getTrigger() {
    return new Trigger(this::isFailing);
  }

  /**
   * Schedules a specified command when the fallible is failing.
   *
   * @param command The command to schedule.
   * @see #getTrigger()
   */
  public default void onFailing(Command command) {
    getTrigger().debounce(DEFAULT_DEBOUNCE_TIME).onTrue(command);
  }

  /**
   * Creates a list of a single fault from necessary information or empty list based on the provided
   * condition.
   *
   * @param description The fault's text.
   * @param type The type of fault.
   * @param condition The condition that defines failure.
   * @return Either a list of one fault or an empty list.
   */
  public default List<Fault> from(String description, FaultType type, boolean condition) {
    return condition ? List.of(new Fault(description, type)) : List.of();
  }

  /**
   * WIP: Returns hardware faults from a {@link CANSparkMax}.
   *
   * @param sparkMax The SparkMax.
   * @return A list of faults containing all reported REVLib errors and faults.
   */
  public default List<Fault> from(CANSparkMax sparkMax) {
    List<Fault> faults = new ArrayList<>();
    REVLibError err = sparkMax.getLastError();
    int id = sparkMax.getDeviceId();
    if (err != REVLibError.kOk) {
      faults.add(
          new Fault(String.format("SparkMax [%d]: Error: %s", id, err.name()), FaultType.ERROR));
    }
    for (FaultID fault : FaultID.values()) {
      if (sparkMax.getFault(fault)) {
        faults.add(
            new Fault(
                String.format("SparkMax [%d]: Fault: %s", id, fault.name()), FaultType.WARNING));
      }
    }
    return faults;
  }

  /**
   * Returns hardware faults from a {@link DutyCycleEncoder}.
   *
   * @param encoder The DutyCycleEncoder.
   * @return A list that is either empty or contains a fault for the encoder being disconnected.
   */
  public default List<Fault> from(DutyCycleEncoder encoder) {
    return from(
        String.format("DutyCycleEncoder [%d]: Disconnected", encoder.getSourceChannel()),
        FaultType.ERROR,
        !encoder.isConnected());
  }

  /**
   * Merges several lists of hardware faults into one.
   *
   * <p>This makes building for {@link #getFaults()} much more ergonomic.
   *
   * @param faults A variable number of lists of faults.
   * @return A single list of faults.
   */
  @SafeVarargs
  public static List<Fault> from(List<Fault>... faults) {
    // calculate length to be allocated
    int len = 0;
    for (List<Fault> f : faults) {
      len += f.size();
    }

    List<Fault> allFaults = new ArrayList<>(len);

    for (List<Fault> f : faults) {
      allFaults.addAll(f);
    }

    return allFaults;
  }

  /**
   * Returns an array of descriptions of all faults that match the specified type.
   *
   * @param type The type to filter for.
   * @return An array of description strings.
   */
  public default String[] getStrings(FaultType type) {
    return getFaults().stream()
        .filter(a -> a.type() == type)
        .map(Fault::description)
        .toArray(String[]::new);
  }

  @Override
  public default void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Alerts");
    builder.addStringArrayProperty("errors", () -> getStrings(FaultType.ERROR), null);
    builder.addStringArrayProperty("warnings", () -> getStrings(FaultType.WARNING), null);
    builder.addStringArrayProperty("infos", () -> getStrings(FaultType.INFO), null);
  }
}
