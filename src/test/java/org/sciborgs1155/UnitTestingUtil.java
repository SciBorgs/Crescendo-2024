package org.sciborgs1155;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;

import static org.sciborgs1155.lib.TestingUtil.runToCompletion;

import org.sciborgs1155.lib.TestingUtil;
import org.sciborgs1155.lib.TestingUtil.Test;
import org.sciborgs1155.lib.TestingUtil.TestBad;

public class UnitTestingUtil {
  public static Command unitTest(Test test) {
    return test.testCommand()
        .apply(true)
        .finallyDo(
            () -> {
              test.truthAssertions()
                  .forEach(a -> assertTrue(a.condition(), a.faultName() + ": " + a.description()));
              test.equalityAssertions()
                  .forEach(
                      a ->
                          assertEquals(
                              a.expected().getAsDouble(),
                              a.actual().getAsDouble(),
                              a.delta(),
                              a.faultName()));
            });
  }

  public static void runUnitTest(Test test) {
    runToCompletion(unitTest(test));
  }

  public static void runUnitTest(TestBad test) {
    runToCompletion(TestingUtil.unitTest(test));
  }
}
