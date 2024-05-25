package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VELOCITY_TOLERANCE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Consumer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
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
  final double SHOOTER_DELTA = 5;

  @BeforeEach
  public void setup() {
    setupTests();
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

  @ParameterizedTest
  @ValueSource(doubles = {-40, -30, 0, 30, 40, 60})
  public void pivotSysCheck(double theta) {
    runUnitTest(pivot.goToTest(Degrees.of(theta)));
  }

  @ParameterizedTest
  @ValueSource(doubles = {200, 300, 350, 400, 500, 540})
  public void shootSysCheck(double v) {
    runUnitTest(shooter.goToTest(RadiansPerSecond.of(v)));
  }

  @ParameterizedTest
  @ValueSource(doubles = {-200, -100, -15, 0, 15, 100, 200})
  public void testShootStoredNote(double vel) {
    run(shooting.shoot(RadiansPerSecond.of(vel)));
    fastForward();

    assertEquals(vel, shooter.rotationalVelocity(), VELOCITY_TOLERANCE.in(RadiansPerSecond));
  }

  @Test
  public void testPivotThenShoot() {
    run(shooting.shootWithPivot(() -> Math.PI / 4, () -> 400));
    fastForward();

    assertEquals(
        MathUtil.clamp(Math.PI / 4, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians)),
        pivot.rotation().getY(),
        DELTA);
    assertEquals(400, shooter.rotationalVelocity(), SHOOTER_DELTA);
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
    testEndCondition.accept(shooting.shootWithPivot(() -> 4, () -> 100));
    testEndCondition.accept(shooting.shoot(RadiansPerSecond.of(150)));
  }
}
