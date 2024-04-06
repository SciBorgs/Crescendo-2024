package org.sciborgs1155.lib;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
}
