package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.UnitTestingUtil.reset;

import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.intake.Intake;

public class IntakeTest {
  @Test
  public void init() throws Exception {
    Intake.create().close();
    reset();
  }
}
