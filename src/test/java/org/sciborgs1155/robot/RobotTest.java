package org.sciborgs1155.robot;

import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.TestingUtil;

public class RobotTest {
  @Test
  void initialize() throws Exception {
    new Robot().close();
    TestingUtil.reset();
  }
}
