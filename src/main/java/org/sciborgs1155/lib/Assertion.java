package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.FaultLogger.FaultType;

public sealed interface Assertion {
  public void apply(boolean unitTest);

  public static class ReportAssertions {
    /**
     * Asserts that a condition is true, and reports to FaultLogger
     *
     * @param condition
     * @param faultName
     * @param description
     */
    private static void assertReport(boolean condition, String faultName, String description) {
      FaultLogger.report(
          faultName,
          (condition ? "success! " : "") + description,
          condition ? FaultType.INFO : FaultType.WARNING);
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
  }

  public static record TruthAssertion(
      BooleanSupplier condition, String faultName, String description) implements Assertion {
    @Override
    public void apply(boolean unitTest) {
      if (unitTest) {
        assertTrue(condition, faultName + ": " + description);
      } else {
        ReportAssertions.assertReport(condition.getAsBoolean(), faultName, description);
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
        ReportAssertions.assertEqualsReport(
            faultName, expected.getAsDouble(), actual.getAsDouble(), delta);
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
