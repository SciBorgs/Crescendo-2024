package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.fastForward;
import static org.sciborgs1155.lib.TestingUtil.run;
import static org.sciborgs1155.lib.TestingUtil.setupHAL;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.shooter.feeder.SimFeeder;
import org.sciborgs1155.robot.shooter.flywheel.SimFlywheel;
import org.sciborgs1155.robot.shooter.pivot.SimPivot;

public class ShooterTest {
  Shooter shooter;
  SimPivot pivot;
  SimFlywheel flywheel;
  SimFeeder feeder;

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
    final double DELTA = 1e-1;
    run(shooter.runFlywheel(() -> 3));
    fastForward(400);

    assertEquals(3, flywheel.getVelocity(), DELTA);
  }

  @Test
  public void testPivot() {
    final double DELTA = 1e-1;
    run((shooter.runPivot(() -> Radians.of(Math.PI / 4))));
    fastForward();

    assertEquals(Math.PI / 4, pivot.getPosition(), DELTA);
  }

  @Test
  public void testClimb() {
    final double DELTA = 1e-1;
    run(shooter.climb(() -> Radians.of(Math.PI / 4)));
    fastForward();
  }

  @Test
  public void testFeeder() {
    final double DELTA = 1e-1;
    run(shooter.runFeeder(2));
    fastForward();

    assertEquals(2, feeder.getVelocity(), DELTA);
  }
}
