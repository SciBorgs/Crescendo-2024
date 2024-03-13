package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Field.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
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
  public Rotation2d findChainAngle(/* Translation2d coordinates */ ) {
    Translation2d nearest;
    Translation2d coords = drive.pose().getTranslation();

    nearest = coords.nearest(chainCoordinateList());

    // TODO forgive me lord for I have sinned
    if (nearest == BLUE_STAGE_AMPSIDE) {
      return Rotation2d.fromRadians(Math.PI * 5 / 3);
    } else if (nearest == BLUE_STAGE_SOURCESIDE) {
      return Rotation2d.fromRadians(Math.PI / 3);
    } else if (nearest == BLUE_STAGE_MIDSIDE) {
      return Rotation2d.fromRadians(Math.PI);
    } else if (nearest == RED_STAGE_AMPSIDE) {
      return Rotation2d.fromRadians(Math.PI * 4 / 3);
    } else if (nearest == RED_STAGE_SOURCESIDE) {
      return Rotation2d.fromRadians(Math.PI * 2 / 3);
    } else if (nearest == RED_STAGE_MIDSIDE) {
      return Rotation2d.fromRadians(0);
    }
    return Rotation2d.fromRadians(0);
  }

  /** Turns the robot such that its heading is perpendicular to its nearest stage climbing chain. */
  public Command snapToStage(
      DoubleSupplier vx, DoubleSupplier vy /*, Supplier<Translation2d> coords*/) {
    return drive.drive(vx, vy, () -> findChainAngle(/* coords */ ));
  }
}
