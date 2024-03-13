package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.drive.Drive;

public class Climbing {
  private final Drive drive;

  public Climbing(Drive drive) {
    this.drive = drive;
  }

  /**
   * @param coordinates current coordinates of the robot as a Translation2d supplier in meters as
   *     units.
   */
  public Rotation2d findChainAngle(Translation2d coordinates) {
    Translation2d nearest;
    if (Constants.alliance() == Alliance.Blue) {
      nearest =
          coordinates.nearest(
              List.of(BLUE_STAGE_AMPSIDE, BLUE_STAGE_SOURCESIDE, BLUE_STAGE_MIDSIDE));
    }
    nearest =
        coordinates.nearest(
            List.of(RED_STAGE_AMPSIDE, RED_STAGE_SOURCESIDE, RED_STAGE_MIDSIDE));

    // TODO forgive me lord for I have sinned
    if (nearest == BLUE_STAGE_AMPSIDE) {
      return Rotation2d.fromRadians(0);
    } else if (nearest == BLUE_STAGE_SOURCESIDE) {
      return Rotation2d.fromRadians(Math.PI * 2 / 3);
    } else if (nearest == BLUE_STAGE_MIDSIDE) {
      return Rotation2d.fromRadians(Math.PI * 4 / 3);
    } else if (nearest == RED_STAGE_AMPSIDE) {
      return Rotation2d.fromRadians(Math.PI * 11 / 6);
    } else if (nearest == RED_STAGE_SOURCESIDE) {
      return Rotation2d.fromRadians(Math.PI / 6);
    } else if (nearest == RED_STAGE_MIDSIDE) {
      return Rotation2d.fromRadians(Math.PI);
    } else {
      throw new NoSuchElementException(
          "Oops. That's not supposed to happen. Climbing did an oopsie");
    }
  }

  /**
   * Turns the robot such that its heading is perpendicular to its nearest stage climbing chain.
   */
  public Command snapToStage(DoubleSupplier vx, DoubleSupplier vy, Supplier<Translation2d> coords) {
    return drive.drive(vx, vy, () -> findChainAngle(coords.get()));
  }
}
