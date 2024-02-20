package org.sciborgs1155.robot;

import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.TestingUtil;
import org.sciborgs1155.robot.intake.Intake;

public class IntakeTest {
  @Test
  public void init() throws Exception {
    var i = Intake.create();
    TestingUtil.closeSubsystem(i);
  }
}
