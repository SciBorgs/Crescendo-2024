package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.shooter.Shooter;

public class ShooterTest {
  Shooter shooter;

  @BeforeEach
  public void setup() {
    setupHAL();
    shooter = Shooter.create();
  }

  @Test
  public void testFlywheel() {
    run(shooter.runFlywheel(() -> 3));
    run(shooter.runPivot(300));
    run(shooter.runFeeder(-0.4));
    fastForward(1000);

    System.out.println(shooter.getFlywheelVelocity());
    assertEquals(3, shooter.getFlywheelVelocity());
    assertEquals(300, shooter.getPivotPosition());
    assertEquals(-0.4, shooter.getFeederVelocity());
  }
}
