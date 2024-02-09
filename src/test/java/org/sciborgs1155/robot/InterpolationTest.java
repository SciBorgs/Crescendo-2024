package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Hashtable;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.shooter.Cache;
import org.sciborgs1155.robot.shooter.Cache.NoteTrajectory;
import org.sciborgs1155.robot.shooter.Shooting;
import org.sciborgs1155.robot.shooter.feeder.Feeder;
import org.sciborgs1155.robot.shooter.flywheel.Flywheel;
import org.sciborgs1155.robot.shooter.pivot.Pivot;

public class InterpolationTest {
  private static Hashtable<Translation2d, NoteTrajectory> data = new Hashtable<>();
  private Shooting shooting;
  private Shooting shootingReal;

  @BeforeAll
  public static void setupData() {
    data.put(new Translation2d(0, 0), new NoteTrajectory(Rotation2d.fromDegrees(30), 2));
    data.put(new Translation2d(0, 2), new NoteTrajectory(Rotation2d.fromDegrees(30), 2));
    data.put(new Translation2d(2, 0), new NoteTrajectory(Rotation2d.fromDegrees(20), 2));
    data.put(new Translation2d(2, 2), new NoteTrajectory(Rotation2d.fromDegrees(20), 2));
  }

  @BeforeEach
  public void setupShooting() {
    shooting = new Shooting(Flywheel.create(), Pivot.create(), Feeder.create(), data);
    shootingReal = new Shooting(Flywheel.create(), Pivot.create(), Feeder.create());
  }

  @Test
  public void interp() throws Exception {
    var state = shooting.desiredState(new Translation2d(1, 1));
    assertEquals(25, state.angle().getDegrees(), 1e-10);
    assertEquals(2, state.speed());
    var state2 = shooting.desiredState(new Translation2d(0.5, 0.5));
    assertEquals(27.5, state2.angle().getDegrees());
  }

  @Test
  public void loads() throws Exception {
    Cache.loadStatesThrows();
  }

  @Test
  public void lookupWithCasadiData() throws Exception {
    var state1 = shootingReal.desiredState(new Translation2d(2, -2));
    assertEquals(0.963, state1.angle().getRadians());
    assertEquals(7.692, state1.speed());

    var state2 = shootingReal.desiredState(new Translation2d(0.5, -0.5));
    assertEquals(1.298, state2.angle().getRadians());
    assertEquals(6.479, state2.speed());
  }

  @Test
  public void fail() throws Exception {
    assertThrows(Exception.class, () -> shooting.desiredState(new Translation2d(3, 1)));
  }
}
