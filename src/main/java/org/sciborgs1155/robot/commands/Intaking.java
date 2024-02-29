package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.vision.NoteVision;

public class Intaking {
  private final Drive drive;
  private final NoteVision noteVision;

  public Intaking(Drive drive, NoteVision noteVision) {
    this.drive = drive;
    this.noteVision = noteVision;
  }

  public Command alignToNearestNote() {
    return drive.driveFacingTarget(() -> 0, () -> 0, () -> noteVision.getNearestNote().get());
  }
}
