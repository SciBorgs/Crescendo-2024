package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.led.Leds;
import org.sciborgs1155.robot.led.Leds.LEDTheme;

public class LedTest {
  Leds led;
  String temp = "";

  @BeforeEach
  public void setup() {
    setupHAL();
    led = new Leds();
  }

  @RepeatedTest(5)
  public void testThemeSet() {
    run(led.setTheme(LEDTheme.SCIBORGS));
    fastForward();
    assertNotEquals(led.getBufferDataString(), temp);
    temp = led.getBufferDataString();
    led.close();
  }

  @RepeatedTest(5)
  public void testRainbowTheme() {
    run(led.setTheme(LEDTheme.RAINBOW));
    fastForward();
    assertNotEquals(led.getBufferData(), temp);
    temp = led.getBufferDataString();
    led.close();
  }
}
