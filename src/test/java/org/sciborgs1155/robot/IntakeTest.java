package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.intake.Intake;

public class IntakeTest {
  Intake intake;

  @BeforeEach
  public void setup() {
    setupHAL();
    intake = new Intake();
  }

  @Test
  public void reachSetpoint() {
    run(intake.spin(true));
    fastForward();
  }

  @AfterEach
  public void close() throws Exception {
    intake.close();
  }
}
