package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.STARTING_ANGLE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VELOCITY_TOLERANCE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Consumer;
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

public class ShootingTest {
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
    reset(pivot, shooter, feeder, drive);
  }

  @Test
  public void testShooter() {
    run(shooter.runShooter(() -> 3));
    fastForward(400);

    assertEquals(3, shooter.rotationalVelocity(), DELTA);
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
        MathUtil.clamp(theta, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians)),
        pivot.rotation().getY(),
        0.15);
  }

  @ParameterizedTest
  @ValueSource(doubles = {1.104793, 3 * Math.PI / 8})
  public void testClimb(double theta) {
    run(pivot.climb(theta));
    fastForward(1000);

    assertEquals(STARTING_ANGLE.in(Radians), pivot.rotation().getY(), DELTA);
  }

  @ParameterizedTest
  @ValueSource(doubles = {-200, -100, -15, 0, 15, 100, 200})
  public void testShootStoredNote(double vel) {
    run(shooting.shoot(RadiansPerSecond.of(vel)));
    fastForward();

    assertEquals(vel, shooter.rotationalVelocity(), VELOCITY_TOLERANCE.in(RadiansPerSecond));
  }

  @Disabled
  @Test
  public void testPivotThenShoot() {
    run(shooting.pivotThenShoot(Radians.of(Math.PI / 4), RadiansPerSecond.of(4)));
    fastForward();

    assertEquals(
        MathUtil.clamp(Math.PI / 4, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians)),
        pivot.rotation().getY(),
        DELTA);
    assertEquals(4, shooter.rotationalVelocity(), DELTA);
  }

  @Test
  public void endConditions() {
    Consumer<Command> testEndCondition =
        c_ -> {
          var c = c_.ignoringDisable(true);
          c.schedule();
          fastForward(10);
          assert !c.isFinished();
        };
    testEndCondition.accept(shooting.pivotThenShoot(Radians.of(4), RadiansPerSecond.of(100)));
    // testEndCondition.accept(shooting.stationaryTurretShooting()); // worked before it was proxied
    testEndCondition.accept(shooting.shoot(RadiansPerSecond.of(150)));
  }
}
