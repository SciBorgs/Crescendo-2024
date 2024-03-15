package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.led.LedStrip;
import org.sciborgs1155.robot.led.LedStrip.LEDTheme;

public class LedTest {
  LedStrip led;
  String temp = "";
  String correct =
      "[#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00]";
  String latest = "";

  @BeforeEach
  public void setup() {
    setupTests();
    led = new LedStrip();
  }

  @Test
  public void testThemeSet() {
    assertEquals(correct, LedStrip.getBufferDataString(LEDTheme.SCIBORGS.ledBuffer.get()));
  }

  @Test
  public void testRainbowTheme() {
    var rainbow1 = LedStrip.getBufferDataString(LEDTheme.RAINBOW.ledBuffer.get());
    fastForward(2);
    var rainbow2 = LedStrip.getBufferDataString(LEDTheme.RAINBOW.ledBuffer.get());
    assertNotEquals(rainbow1, rainbow2);
  }

  @AfterEach
  public void destroy() {
    led.close();
  }
}
