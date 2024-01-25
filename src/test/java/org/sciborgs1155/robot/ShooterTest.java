package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.fastForward;
import static org.sciborgs1155.lib.TestingUtil.run;
import static org.sciborgs1155.lib.TestingUtil.setupHAL;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.shooter.feeder.SimFeeder;
import org.sciborgs1155.robot.shooter.flywheel.SimFlywheel;
import org.sciborgs1155.robot.shooter.pivot.SimPivot;

public class ShooterTest {
  Shooter shooter;
  SimPivot pivot;
  SimFlywheel flywheel;
  SimFeeder feeder;

  final double DELTA = 1e-1;

  @BeforeEach
  public void setup() {
    setupHAL();
    pivot = new SimPivot();
    flywheel = new SimFlywheel();
    feeder = new SimFeeder();
    shooter = new Shooter(flywheel, pivot, feeder);
  }

  @Test
  public void testFlywheel() {
    run(shooter.runFlywheel(() -> 3));
    fastForward(400);

    assertEquals(3, flywheel.getVelocity(), DELTA);
  }

  @ParameterizedTest
  @ValueSource(doubles = {Math.PI / 4, Math.PI / 8, 3 * Math.PI / 8})
  public void testPivot(double theta) {
    run((shooter.runPivot(() -> Radians.of(Math.PI / 4))));
    fastForward(600);

    assertEquals(Math.PI / 4, pivot.getPosition(), DELTA);
  }

  @Disabled
  @Test
  public void testClimb() {
    run(shooter.climb(() -> Radians.of(Math.PI / 4)));
    fastForward();

    assertEquals(Math.PI / 4, Math.PI);
  }

  @Test
  public void testFeeder() {
    run(shooter.runFeeder(2));
    fastForward();

    assertEquals(2, feeder.getVelocity(), DELTA);
  }

  @Disabled
  @Test
  public void testShootStoredNote() {
    run(shooter.shootStoredNote(() -> 4));
    fastForward();

    assertEquals(4, flywheel.getVelocity(), DELTA);
    assertEquals(1, feeder.getVelocity(), DELTA);
  }
}
