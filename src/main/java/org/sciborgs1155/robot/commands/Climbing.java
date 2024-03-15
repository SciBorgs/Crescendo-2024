package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Constants.Field;
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
  public Rotation2d findChainAngle() {
    return drive.pose().nearest(Field.chainCoordinates()).getRotation();
  }

  /** Turns the robot such that its heading is perpendicular to its nearest stage climbing chain. */
  public Command snapToStage(DoubleSupplier vx, DoubleSupplier vy) {
    return drive.drive(vx, vy, () -> findChainAngle());
  }
}
