package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.pivot.PivotConstants.PRESET_CLIMBING_ANGLE;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Constants.Field;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.pivot.Pivot;

public class Climbing {
  private final Drive drive;
  private final Pivot pivot;

  public Climbing(Drive drive, Pivot pivot) {
    this.drive = drive;
    this.pivot = pivot;
  }

  /** returns the angle at which the robot will be facing perpendicular to the nearest chain. */
  public Rotation2d findChainAngle() {
    return drive.pose().nearest(Field.chainCoordinates()).getRotation();
  }

  /**
   * Turns the robot such that its heading is perpendicular to its nearest stage climbing chain.
   * Additionally, while held, moves the pivot arm to the required angle for the arm to latch onto
   * the chain.
   */
  public Command snapToStage(DoubleSupplier vx, DoubleSupplier vy) {
    return drive.drive(vx, vy, this::findChainAngle);
  }

  /** moves the pivot arm to the required angle for the arm to latch onto the chain. */
  public Command angleClimber() {
    return pivot.runPivot(PRESET_CLIMBING_ANGLE);
  }
}
