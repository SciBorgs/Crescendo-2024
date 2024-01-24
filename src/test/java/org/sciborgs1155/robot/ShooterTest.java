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
  final double DELTA = 1e-2;

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
    run(shooter.runPivot(() -> Radians.of(Math.PI / 4)));
    run(shooter.runPivot(() -> Radians.of(Math.PI / 4)));
    run(shooter.runFeeder(-0.4));
    fastForward(1000);

    assertEquals(3, flywheel.getVelocity(), DELTA);
    assertEquals(300, pivot.getPosition(), DELTA);
    assertEquals(-0.4, feeder.getVelocity(), DELTA);
  }
}
