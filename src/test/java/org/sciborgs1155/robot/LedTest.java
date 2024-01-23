package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;
import org.sciborgs1155.robot.led.Leds;
import org.sciborgs1155.robot.led.Leds.LEDTheme;


import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


public class LedTest {
  Leds led = new Leds();
  String correct = "[#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00,#008000,#FFFF00]";
  
  @BeforeEach
  public void setup() {
    setupHAL();
  }
  
  @Test
  public void testThemeSet() {
    run(led.setTheme(LEDTheme.BXSCI));
    fastForward();
    assertEquals(led.getBufferDataString(), correct);
  }
}
