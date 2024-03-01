package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.vision.NoteVision;

public class Intaking {
  private final Drive drive;
  private final NoteVision noteVision;

  public Intaking(Drive drive, NoteVision noteVision) {
    this.drive = drive;
    this.noteVision = noteVision;
  }

  public Command alignToNearestNote(DoubleSupplier vx, DoubleSupplier vy) {
    return drive.driveFacingTarget(vx, vy, noteVision::note);
  }
}
