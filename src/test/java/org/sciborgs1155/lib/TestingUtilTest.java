package org.sciborgs1155.lib;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.TestingUtil.Assertion.eAssert;
import static org.sciborgs1155.lib.TestingUtil.Assertion.tAssert;
import static org.sciborgs1155.lib.TestingUtil.runToCompletion;
import static org.sciborgs1155.lib.TestingUtil.runUnitTest;
import static org.sciborgs1155.lib.TestingUtil.setupTests;
import static org.sciborgs1155.lib.TestingUtil.systemsCheck;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.stream.Collectors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.lib.FaultLogger.Fault;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.TestingUtil.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.TestingUtil.Assertion.TruthAssertion;
import org.sciborgs1155.lib.TestingUtil.Test;

public class TestingUtilTest {
  int x;

  @BeforeEach
  public void setup() {
    setupTests();
    x = 0;
  }

  @AfterEach
  public void clear() throws Exception {
    TestingUtil.reset();
    x = 0;
  }

  public void increment() {
    x += 1;
  }

  public void set(int x) {
    this.x = x;
  }

  @org.junit.jupiter.api.Test
  public void enabled() {
    assertTrue(DriverStation.isEnabled());
  }

  @org.junit.jupiter.api.Test
  public void testFromCommand() throws Exception {
    assertEquals(0, x);
    Test t = Test.fromCommand(Commands.runOnce(this::increment));
    Command c = systemsCheck(t);
    runToCompletion(c);
    assertEquals(1, x);
  }

  @ParameterizedTest
  @ValueSource(doubles = {0.4, 2, 3.2, 4.03})
  public void runToCompletionTest(double timeout) {
    Command c = Commands.run(() -> {}).withTimeout(timeout);
    double startTime = Timer.getFPGATimestamp();
    runToCompletion(c);
    assertEquals(timeout, Timer.getFPGATimestamp() - startTime, 0.3);
  }

  public void assertFaultCount(int infoCount, int warningCount, int errorCount) {
    FaultLogger.update();
    Set<Fault> faults = FaultLogger.totalFaults();
    Set<Fault> infos =
        faults.stream().filter(f -> f.type() == FaultType.INFO).collect(Collectors.toSet());
    Set<Fault> warnings =
        faults.stream().filter(f -> f.type() == FaultType.WARNING).collect(Collectors.toSet());
    Set<Fault> errors =
        faults.stream().filter(f -> f.type() == FaultType.ERROR).collect(Collectors.toSet());
    assertEquals(infoCount, infos.size(), infos.toString());
    assertEquals(warningCount, warnings.size());
    assertEquals(errorCount, errors.size());
  }

  @ParameterizedTest
  @ValueSource(ints = {-4, 3, 9})
  public void systemCheck(int x) {
    EqualityAssertion goodAssertion = eAssert("x", () -> x, () -> this.x);
    Test passes = new Test(runOnce(() -> set(x)), Set.of(goodAssertion));
    runToCompletion(systemsCheck(passes));
    assertFaultCount(1, 0, 0);

    TruthAssertion badAssertion = tAssert(() -> x != this.x, "x", "fails");
    Test fails = new Test(runOnce(() -> set(x)), Set.of(badAssertion));
    runToCompletion(systemsCheck(fails));
    assertFaultCount(1, 1, 0);

    FaultLogger.clear();
    FaultLogger.unregisterAll();
    Test combo = new Test(runOnce(() -> set(x)), Set.of(goodAssertion, badAssertion));
    runToCompletion(systemsCheck(combo));
    assertFaultCount(1, 1, 0);
  }

  @ParameterizedTest
  @ValueSource(ints = {-4, 3, 9})
  public void unitTest(int x) {
    EqualityAssertion goodAssertion = eAssert("x", () -> x, () -> this.x);
    Test passes = new Test(runOnce(() -> set(x)), Set.of(goodAssertion));
    runUnitTest(passes);
    assertFaultCount(0, 0, 0);

    TruthAssertion badAssertion = tAssert(() -> x != this.x, "x", "fails");
    Test fails = new Test(runOnce(() -> set(x)), Set.of(badAssertion));
    try {
      runUnitTest(fails);
      assert false;
    } catch (Error e) {
    }
    assertFaultCount(0, 0, 0);
  }
}
