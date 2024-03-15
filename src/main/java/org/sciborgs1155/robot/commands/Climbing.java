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

  /** returns the angle at which the robot will be facing perpendicular to the nearest chain. */
  public Rotation2d findChainAngle() {
    return drive.pose().nearest(Field.chainCoordinates()).getRotation();
  }

  /** Turns the robot such that its heading is perpendicular to its nearest stage climbing chain. */
  public Command snapToStage(DoubleSupplier vx, DoubleSupplier vy) {
    return drive.drive(vx, vy, this::findChainAngle);
  }
}
