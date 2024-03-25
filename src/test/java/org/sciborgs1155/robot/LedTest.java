package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.TestingUtil;
import org.sciborgs1155.robot.led.LedConstants;
import org.sciborgs1155.robot.led.LedStrip;
import org.sciborgs1155.robot.led.LedStrip.LEDTheme;

public class LedTest {
  LedStrip led;
  String temp = "";
  String latest = "";

  @BeforeEach
  public void setup() {
    setupTests();
    led = new LedStrip();
  }

  @Test
  public void testThemeSet() {
    String correct = "[";
    for (int i = 0; i < LedConstants.LEDLENGTH - 1; i++) {
      correct += (i % 2 == 0) ? "#FFFF00," : "#A9A9A9,";
    }
    correct += (LedConstants.LEDLENGTH % 2 == 0) ? "#A9A9A9]" : "#FFFF00]";

    assertEquals(correct, LedStrip.getBufferDataString(LEDTheme.TEST.ledBuffer.get()));
  }

  @Test
  public void testRainbowTheme() {
    var rainbow1 = LedStrip.getBufferDataString(LEDTheme.RAINBOW.ledBuffer.get());
    fastForward(2);
    var rainbow2 = LedStrip.getBufferDataString(LEDTheme.RAINBOW.ledBuffer.get());
    assertNotEquals(rainbow1, rainbow2);
  }

  @AfterEach
  public void destroy() throws Exception {
    TestingUtil.reset(led);
  }
}
