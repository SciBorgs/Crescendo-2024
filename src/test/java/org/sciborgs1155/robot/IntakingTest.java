package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.TestingUtil.setupHAL;

import org.junit.jupiter.api.BeforeEach;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.intake.Intake;
import org.sciborgs1155.robot.vision.NoteVision;

public class IntakingTest {
  Drive drive;
  Intake intake;
  NoteVision noteVision;

  @BeforeEach
  public void setup() {
    setupHAL();

    drive = Drive.create();
    intake = Intake.create();
    noteVision = new NoteVision();
  }
}
