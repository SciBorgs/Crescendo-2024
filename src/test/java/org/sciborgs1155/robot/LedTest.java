package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.led.Leds;
import org.sciborgs1155.robot.led.Leds.LEDTheme;

public class LedTest {
  Leds led;
  String temp = "";
  String correct =
      "[#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00]";

  @BeforeEach
  public void setup() {
    setupHAL();
    led = new Leds();
  }

  @RepeatedTest(1)
  public void testThemeSet() {
    run(led.setLEDTheme(LEDTheme.SCIBORGS));
    fastForward();
    assertEquals(led.getBufferDataString(), correct);
    led.close();
  }

  @RepeatedTest(1)
  public void testRainbowTheme() {
    run(led.setLEDTheme(LEDTheme.RAINBOW));
    fastForward();
    assertNotEquals(led.getBufferData(), temp);
    temp = led.getBufferDataString();
    led.close();
  }
}
