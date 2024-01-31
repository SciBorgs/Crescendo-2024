package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.fastForward;
import static org.sciborgs1155.lib.TestingUtil.run;
import static org.sciborgs1155.lib.TestingUtil.setupHAL;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.shooter.Shooting;
import org.sciborgs1155.robot.shooter.feeder.Feeder;
import org.sciborgs1155.robot.shooter.flywheel.Flywheel;
import org.sciborgs1155.robot.shooter.pivot.Pivot;

public class ShooterTest {
  Shooting shooter;
  Pivot pivot;
  Flywheel flywheel;
  Feeder feeder;

  final double DELTA = 1e-1;

  @BeforeEach
  public void setup() {
    setupHAL();
    pivot = Pivot.create();
    flywheel = Flywheel.create();
    feeder = Feeder.create();
    shooter = new Shooting(flywheel, pivot, feeder);
  }

  @Test
  public void testFlywheel() {
    run(flywheel.runFlywheel(() -> 3));
    fastForward(400);

    assertEquals(3, flywheel.getVelocity(), DELTA);
  }

  @Disabled
  @ParameterizedTest
  @ValueSource(doubles = {Math.PI / 4, 3 * Math.PI / 8})
  public void testPivot(double theta) {
    run((pivot.runPivot(() -> Rotation2d.fromRadians(theta))));
    assertEquals(pivot.pivotPID.getGoal().position, theta);
    fastForward(3000);
    assertEquals(pivot.pivotPID.getGoal().position, theta);
    assertEquals(pivot.pivotPID.getGoal().velocity, 0);
    assertEquals(pivot.pivotPID.getSetpoint().position, theta);
    assertEquals(pivot.pivotPID.getSetpoint().velocity, 0);
    assertEquals(theta, pivot.getPosition().getRadians(), 0.15);
  }

  @Test
  public void testClimb() {
    run(pivot.climb(() -> new Rotation2d(Radians.of(Math.PI / 4))));
    fastForward();

    assertEquals(Math.PI / 4, pivot.getPosition().getRadians(), DELTA);
  }

  @Test
  public void testFeeder() {
    // this is just making sure it doesn't kill itself
    run(feeder.runFeeder(4));
    fastForward();
    assertEquals("running feeder, " + 4. + " volts", feeder.getCurrentCommand().getName());
  }

  @Test
  public void testShootStoredNote() {
    run(shooter.shootStoredNote(() -> 4));
    fastForward();

    assertEquals(4, flywheel.getVelocity(), DELTA);
  }

  @Disabled
  @Test
  public void testPivotThenShoot() {
    run(shooter.pivotThenShoot(() -> new Rotation2d(Radians.of(Math.PI / 4)), () -> 4));
    fastForward();

    assertEquals(Math.PI / 4, pivot.getPosition().getRadians(), DELTA);
    assertEquals(4, flywheel.getVelocity(), DELTA);
  }
}
