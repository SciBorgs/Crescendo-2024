package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.intake.Intake;
import org.sciborgs1155.robot.vision.NoteVision;

public class Intaking {
  private final Drive drive;
  private final Intake intake;
  private final NoteVision noteVision;

  public Intaking(Drive drive, Intake intake, NoteVision noteVision) {
    this.drive = drive;
    this.intake = intake;
    this.noteVision = noteVision;
  }

  public Command alignToNearestNote(DoubleSupplier vx, DoubleSupplier vy) {
    return drive.driveFacingTarget(vx, vy, () -> noteVision.note());
  }

  public Command intaking(DoubleSupplier vx, DoubleSupplier vy) {
    return intake
        .intake()
        .deadlineWith(alignToNearestNote(vx, vy))
        .andThen(new RunCommand(noteVision::resetLast));
  }
}
