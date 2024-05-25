package org.sciborgs1155.lib;

import static org.sciborgs1155.lib.UnitTestingUtil.runToCompletion;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;

public record Test(Command testCommand, Set<Assertion> assertions) {
  /**
   * @param command
   * @return a Test with no assertions
   */
  public static Test fromCommand(Command command) {
    return new Test(command, Set.of());
  }

  /** Creates a command from a Test */
  public static Command toCommand(Test test) {
    return test.testCommand.finallyDo(() -> test.assertions.forEach(a -> a.apply(false)));
  }

  /** Creates a sequential command from Tests. */
  public static Command toCommand(Test... tests) {
    Command c = Commands.none();
    for (Test test : tests) {
      c = c.andThen(toCommand(test));
    }
    return c;
  }

  private static Command toUnitTestCommand(Test test) {
    return test.testCommand.finallyDo(() -> test.assertions.forEach(a -> a.apply(true)));
  }

  /**
   * Runs a unit test based on a Test.
   *
   * @param test
   */
  public static void runUnitTest(Test test) {
    runToCompletion(toUnitTestCommand(test));
  }
}
