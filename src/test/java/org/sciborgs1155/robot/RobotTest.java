package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.UnitTestingUtil.reset;

import org.junit.jupiter.api.Test;

public class RobotTest {
  @Test
  void initialize() throws Exception {
    new Robot().close();
    reset();
  }
}
