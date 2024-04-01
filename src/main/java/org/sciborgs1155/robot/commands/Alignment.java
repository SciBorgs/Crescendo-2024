package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static org.sciborgs1155.robot.Constants.Field.amp;
import static org.sciborgs1155.robot.pivot.PivotConstants.AMP_ANGLE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Constants.Field;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.pivot.Pivot;

public class Alignment {
  private final Drive drive;
  private final Pivot pivot;

  public Alignment(Drive drive, Pivot pivot) {
    this.drive = drive;
    this.pivot = pivot;
  }

  /**
   * Turns the robot such that the shooter is pointing into the amp opening. Additionally, while
   * held, moves the pivot arm to the required angle for the note to go into the amp.
   */
  public Command ampAlign() {
    return drive
        .driveTo(
            new Pose2d(
                amp()
                    .plus(
                        new Translation2d(
                            Inches.of(0),
                            DriveConstants.CHASSIS_WIDTH.times(-0.5).plus(Centimeters.of(8)))),
                Rotation2d.fromRadians(-Math.PI / 2)))
        .deadlineWith(
            Commands.waitUntil(() -> drive.pose().getTranslation().getDistance(Field.amp()) < 1)
                .andThen(pivot.runPivot(AMP_ANGLE)));
  }

  /** returns the angle at which the robot will be facing perpendicular to the nearest chain. */
  public Rotation2d angleToChain() {
    return drive.pose().nearest(Field.chain()).getRotation();
  }

  /**
   * Turns the robot such that its heading is perpendicular to its nearest stage climbing chain.
   * Additionally, while held, moves the pivot arm to the required angle for the arm to latch onto
   * the chain.
   */
  public Command snapToStage(DoubleSupplier vx, DoubleSupplier vy) {
    return drive.drive(vx, vy, this::angleToChain);
  }
}
