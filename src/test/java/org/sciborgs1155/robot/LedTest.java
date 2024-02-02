package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
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
    setupHAL();
    led = new LedStrip();
  }

  @RepeatedTest(1)
  public void testThemeSet() {
    run(led.setLEDTheme(LEDTheme.SCIBORGS));
    fastForward();
    assertEquals(correct, LedStrip.getBufferDataString(LEDTheme.SCIBORGS.ledBuffer.get()));
    led.close();
  }

  @RepeatedTest(1)
  public void testRainbowTheme() {
    run(led.setLEDTheme(LEDTheme.RAINBOW));
    fastForward();
    latest = LedStrip.getBufferDataString(LEDTheme.RAINBOW.ledBuffer.get());
    assertNotEquals(temp,latest);
    temp = latest;
    led.close();
  }
}
