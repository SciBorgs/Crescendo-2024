package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.STARTING_ANGLE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VELOCITY_TOLERANCE;

import edu.wpi.first.math.MathUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.commands.Shooting;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.shooter.Shooter;

public class ShooterTest {
  Shooting shooting;
  Pivot pivot;
  Shooter shooter;
  Feeder feeder;
  Drive drive;

  final double DELTA = 1e-1;

  @BeforeEach
  public void setup() {
    setupHAL();
    pivot = Pivot.create();
    shooter = Shooter.create();
    feeder = Feeder.create();
    drive = Drive.create();
    shooting = new Shooting(shooter, pivot, feeder, drive);
  }

  @AfterEach
  public void destroy() throws Exception {
    // reset(pivot, shooter, feeder, drive);
  }

  @Test
  public void testShooter() {
    run(shooter.runShooter(() -> 3));
    fastForward(400);

    assertEquals(3, shooter.getVelocity(), DELTA);
  }

  @Disabled
  @ParameterizedTest
  @ValueSource(doubles = {1.104793, 3 * Math.PI / 8})
  public void testPivot(double theta) {
    run((pivot.runPivot(() -> theta)));
    fastForward(200);
    assertEquals(theta, pivot.goal().getY(), DELTA);
    assertEquals(theta, pivot.setpoint().getY(), DELTA);
    assertEquals(
        MathUtil.clamp(theta, MIN_ANGLE.getRadians(), MAX_ANGLE.getRadians()),
        pivot.rotation().getY(),
        0.15);
  }

  @ParameterizedTest
  @ValueSource(doubles = {1.104793, 3 * Math.PI / 8})
  public void testClimb(double theta) {
    run(pivot.climb(theta));
    fastForward(1000);

    assertEquals(STARTING_ANGLE.getRadians(), pivot.rotation().getY(), DELTA);
  }

  @ParameterizedTest
  @ValueSource(doubles = {-200, -100, -15, 0, 15, 100, 200})
  public void testShootStoredNote(double vel) {
    run(shooting.shoot(vel));
    fastForward();

    assertEquals(vel, shooter.getVelocity(), VELOCITY_TOLERANCE.in(RadiansPerSecond));
  }

  @Disabled
  @Test
  public void testPivotThenShoot() {
    run(shooting.pivotThenShoot(Math.PI / 4, 4));
    fastForward();

    assertEquals(
        MathUtil.clamp(Math.PI / 4, MIN_ANGLE.getRadians(), MAX_ANGLE.getRadians()),
        pivot.rotation().getY(),
        DELTA);
    assertEquals(4, shooter.getVelocity(), DELTA);
  }

  @Test
  public void endConditions() {
    var c = shooting.pivotThenShoot(4, 5).ignoringDisable(true);
    c.schedule();
    fastForward(10);
    assert !c.isFinished();
    var c2 = shooting.stationaryTurretShooting().ignoringDisable(true);
    c2.schedule();
    fastForward(2);
    assert !c2.isFinished();
    var c3 = shooting.stationaryShooting().ignoringDisable(true);
    c3.schedule();
    fastForward(2);
    assert !c3.isFinished();
    var c4 = shooting.shoot(3, pivot::atGoal).ignoringDisable(true);
    c4.schedule();
    fastForward(2);
    assert !c4.isFinished();
  }
}
