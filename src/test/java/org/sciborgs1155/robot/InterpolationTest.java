package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Hashtable;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.shooter.Shooting;
import org.sciborgs1155.robot.shooter.Shooting.ShooterState;
import org.sciborgs1155.robot.shooter.feeder.Feeder;
import org.sciborgs1155.robot.shooter.flywheel.Flywheel;
import org.sciborgs1155.robot.shooter.pivot.Pivot;

public class InterpolationTest {
  private Hashtable<Translation2d, ShooterState> data = new Hashtable<>();
  private Shooting shooting;

  @BeforeEach
  public void setup() {
    data.put(new Translation2d(0, 0), new ShooterState(Rotation2d.fromDegrees(30), 2));
    data.put(new Translation2d(0, 2), new ShooterState(Rotation2d.fromDegrees(30), 2));
    data.put(new Translation2d(2, 0), new ShooterState(Rotation2d.fromDegrees(20), 2));
    data.put(new Translation2d(2, 2), new ShooterState(Rotation2d.fromDegrees(20), 2));

    shooting = new Shooting(Flywheel.create(), Pivot.create(), Feeder.create(), data);
  }

  @Test
  public void interp() throws Exception {
    var state = shooting.desiredState(new Translation2d(1, 1));
    assertEquals(25, state.angle().getDegrees(), 1e-10);
    assertEquals(2, state.velocity());
  }
}
