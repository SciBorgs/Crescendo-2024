package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.*;
import static org.sciborgs1155.lib.TestingUtil.Assertion.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.sciborgs1155.lib.FaultLogger.FaultType;

public class TestingUtil {
  /**
   * Runs CommandScheduler repeatedly to fast forward subsystems and run commands.
   *
   * @param ticks The number of times CommandScheduler is run
   */
  public static void fastForward(int ticks) {
    for (int i = 0; i < ticks; i++) {
      CommandScheduler.getInstance().run();
    }
  }

  /** Runs CommandScheduler 200 times to fast forward subsystems and run commands. */
  public static void fastForward() {
    fastForward(200);
  }

  /**
   * Schedules and runs a command while disabled
   *
   * @param command The command to run.
   */
  public static void run(Command command) {
    command.schedule();
    CommandScheduler.getInstance().run();
  }

  /**
   * Schedules and runs a command while disabled.
   *
   * @param command The command to run.
   * @param runs The number of times CommandScheduler is run.
   */
  public static void run(Command command, int runs) {
    command.schedule();
    fastForward(runs);
  }

  public static void runToCompletion(Command command) {
    command.schedule();
    while (command.isScheduled()) {
      fastForward(1);
    }
  }

  /** Sets up DS and initializes HAL with default values and asserts that it doesn't fail. */
  public static void setupTests() {
    assert HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setTest(true);
    DriverStationSim.notifyNewData();
  }

  /**
   * Resets CommandScheduler and closes all subsystems. Please call in an @AfterEach method!
   *
   * @param subsystems All subsystems that need to be closed
   */
  public static void reset(AutoCloseable... subsystems) throws Exception {
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().cancelAll();
    for (AutoCloseable subsystem : subsystems) {
      subsystem.close();
    }
  }

  public static void assertEqualsReport(
      String faultName, double expected, double actual, double delta) {
    assertReport(
        Math.abs(expected - actual) <= delta,
        faultName,
        "expected: " + expected + "; actual: " + actual);
  }

  public static void assertReport(boolean condition, String faultName, String description) {
    FaultLogger.report(
        faultName,
        (condition ? "success! " : "") + description,
        condition ? FaultType.INFO : FaultType.WARNING);
  }

  public sealed interface Assertion {
    public void apply(boolean unitTest);

    public static record TruthAssertion(
        BooleanSupplier condition, String faultName, String description) implements Assertion {
      @Override
      public void apply(boolean unitTest) {
        if (unitTest) {
          assertTrue(condition, faultName + ": " + description);
        } else {
          assertReport(condition.getAsBoolean(), faultName, description);
        }
      }
    }

    public static record EqualityAssertion(
        String faultName, DoubleSupplier expected, DoubleSupplier actual, double delta)
        implements Assertion {
      @Override
      public void apply(boolean unitTest) {
        if (unitTest) {
          assertEquals(expected.getAsDouble(), actual.getAsDouble(), delta, faultName);
        }
        assertEqualsReport(faultName, expected.getAsDouble(), actual.getAsDouble(), delta);
      }
    }
  }

  public static TruthAssertion tAssert(
      BooleanSupplier condition, String faultName, String description) {
    return new TruthAssertion(condition, faultName, description);
  }

  public static EqualityAssertion eAssert(
      String faultName, DoubleSupplier expected, DoubleSupplier actual, double delta) {
    return new EqualityAssertion(faultName, expected, actual, delta);
  }

  public static record Test(Function<Boolean, Command> testCommand, Set<Assertion> assertions) {}

  // i hate this name but it makes something else nicer idk
  public static Test genTest(Command command) {
    return new Test(b -> command, Set.of());
  }

  public static Command systemsCheck(Test test) {
    return test.testCommand
        .apply(false)
        .finallyDo(() -> test.assertions.forEach(a -> a.apply(false)));
  }

  public static Command systemsCheck(Test... tests) {
    Command c = Commands.none();
    for (Test test : tests) {
      c = c.andThen(systemsCheck(test));
    }
    return c;
  }

  public static Command unitTest(Test test) {
    return test.testCommand
        .apply(true)
        .finallyDo(() -> test.assertions.forEach(a -> a.apply(true)));
  }

  public static void runUnitTest(Test test) {
    runToCompletion(unitTest(test));
  }
}
