package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.FaultLogger.FaultType;

public class TestingUtil {
  public static final Measure<Time> TICK_RATE = Seconds.of(0.02);

  public static record Test(Command testCommand, Set<Assertion> assertions) {
    /**
     * @param command
     * @return a Test with no assertions
     */
    public static Test fromCommand(Command command) {
      return new Test(command, Set.of());
    }
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
        } else {
          assertEqualsReport(faultName, expected.getAsDouble(), actual.getAsDouble(), delta);
        }
      }
    }

    /**
     * @return a truth assertion
     */
    public static TruthAssertion tAssert(
        BooleanSupplier condition, String faultName, String description) {
      return new TruthAssertion(condition, faultName, description);
    }

    /**
     * @return an equality assertion
     */
    public static EqualityAssertion eAssert(
        String faultName, DoubleSupplier expected, DoubleSupplier actual, double delta) {
      return new EqualityAssertion(faultName, expected, actual, delta);
    }

    /**
     * @return an equality assertion
     */
    public static EqualityAssertion eAssert(
        String faultName, DoubleSupplier expected, DoubleSupplier actual) {
      return eAssert(faultName, expected, actual, 0);
    }
  }

  /**
   * Runs CommandScheduler and updates timer repeatedly to fast forward subsystems and run commands.
   *
   * @param ticks The number of times CommandScheduler is run
   */
  public static void fastForward(int ticks) {
    for (int i = 0; i < ticks; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(0.02);
    }
  }

  /**
   * Fasts forward in time by running CommandScheduler and updating timer
   *
   * @param time
   */
  public static void fastForward(Measure<Time> time) {
    fastForward((int) (time.in(Seconds) / TICK_RATE.in(Seconds)));
  }

  /**
   * Runs CommandScheduler and upates timer 200 times to fast forward subsystems and run commands.
   */
  public static void fastForward() {
    fastForward(200);
  }

  /**
   * Schedules and runs a command
   *
   * @param command The command to run.
   */
  public static void run(Command command) {
    command.schedule();
    CommandScheduler.getInstance().run();
  }

  /**
   * Schedules and runs a command.
   *
   * @param command The command to run.
   * @param runs The number of times CommandScheduler is run.
   */
  public static void run(Command command, int runs) {
    command.schedule();
    fastForward(runs);
  }

  /**
   * Schedules a command and runs it until it ends
   *
   * @param command
   */
  public static void runToCompletion(Command command) {
    command.schedule();
    fastForward(1);
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
    FaultLogger.clear();
    FaultLogger.unregisterAll();
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

  /**
   * Asserts that two values are equal (with some tolerance), and reports to FaultLogger
   *
   * @param faultName
   * @param expected
   * @param actual
   * @param delta tolerance
   */
  public static void assertEqualsReport(
      String faultName, double expected, double actual, double delta) {
    assertReport(
        Math.abs(expected - actual) <= delta,
        faultName,
        "expected: " + expected + "; actual: " + actual);
  }

  /**
   * Asserts that a condition is true, and reports to FaultLogger
   *
   * @param condition
   * @param faultName
   * @param description
   */
  public static void assertReport(boolean condition, String faultName, String description) {
    FaultLogger.report(
        faultName,
        (condition ? "success! " : "") + description,
        condition ? FaultType.INFO : FaultType.WARNING);
  }

  /** Creates a systems check command from a Test. */
  public static Command systemsCheck(Test test) {
    return test.testCommand.finallyDo(() -> test.assertions.forEach(a -> a.apply(false)));
  }

  /** Creates a systems check sequential command from Tests. */
  public static Command systemsCheck(Test... tests) {
    Command c = Commands.none();
    for (Test test : tests) {
      c = c.andThen(systemsCheck(test));
    }
    return c;
  }

  private static Command unitTest(Test test) {
    return test.testCommand.finallyDo(() -> test.assertions.forEach(a -> a.apply(true)));
  }

  /**
   * Runs a unit test based on a Test.
   *
   * @param test
   */
  public static void runUnitTest(Test test) {
    runToCompletion(unitTest(test));
  }
}
