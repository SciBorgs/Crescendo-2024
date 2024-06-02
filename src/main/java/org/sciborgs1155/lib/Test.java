package org.sciborgs1155.lib;

import static org.sciborgs1155.lib.UnitTestingUtil.runToCompletion;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;

/** A test, consisting of a command and assertions. Can be used for unit tests or ran on robot. */
public record Test(Command testCommand, Set<Assertion> assertions) {
  /**
   * @param command
   * @return a Test with no assertions
   */
  public static Test fromCommand(Command command) {
    return new Test(command, Set.of());
  }

  private static Command toCommand(Test test, boolean unitTest) {
    return test.testCommand.finallyDo(() -> test.assertions.forEach(a -> a.apply(unitTest)));
  }

  /**
   * Creates a command from a Test
   *
   * @param test a Test
   * @return a command that runs the testCommand in test and then applies all assertions
   */
  public static Command toCommand(Test test) {
    return toCommand(test, false);
  }

  /**
   * Creates a sequential command from Tests.
   *
   * @return a command that runs the testCommand and assertions from each test in turn
   */
  public static Command toCommand(Test... tests) {
    Command c = Commands.none();
    for (Test test : tests) {
      c = c.andThen(toCommand(test));
    }
    return c;
  }

  /**
   * Runs a unit test based on a Test.
   *
   * @param test
   */
  public static void runUnitTest(Test test) {
    runToCompletion(toCommand(test, true));
  }
}
