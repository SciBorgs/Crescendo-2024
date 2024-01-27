package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.led.Leds;
import org.sciborgs1155.robot.led.Leds.LEDTheme;

public class LedTest {
  Leds led;
  String correct =
      "[#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00,#A9A9A9,#FFFF00]";

  @BeforeEach
  public void setup() {
    setupHAL();
    led = new Leds();
  }

  @Test
  public void testThemeSet() {
    run(led.setTheme(LEDTheme.SCIBORGS));
    fastForward();
    assertEquals(led.getBufferDataString(), correct);
  }
}
