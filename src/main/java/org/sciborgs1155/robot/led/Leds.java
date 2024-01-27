package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Constants.Led.LEDLENGTH;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Leds extends SubsystemBase implements Logged, AutoCloseable {
  private final AddressableLED led = new AddressableLED(0); // led as a class
  public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);

  public static enum LEDTheme {
    RAINBOW, // RGB Gamer Robot
    BXSCI, // Yellow 50%, Green 50%
    BXSCIFLASH, // Yellow 20%, Green 80%, Moving,
    // (All Yellow moves in one direction, yellow every 5 LEDs, green rest)
    IN_INTAKE, // Look at Constants, Orange 100%
    IN_PASSING, // Look at Constants, Grey 100% (but they write it as gray)
    IN_SHOOTER, // Look at Constants, Green 100%
    AUTO, // Yellow 50%, Black 50%
    ERROR, // Red 100%
    EXPLODE // Red 100%
  }

  static double ticktime = 0;

  public Leds() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void setLEDTheme(LEDTheme ledTheme) {
    if (ledTheme == LEDTheme.RAINBOW) {
      ticktime += 16; // 1 tick = 0.005 seconds    200 ticks = 1 second
      for (int i = 0; i < ledBuffer.getLength(); i++) {

        final double constant = i / (ledBuffer.getLength() * (Math.PI / 2));
        double green = Math.sin(ticktime / 200 + (constant));
        double blue = Math.cos(ticktime / 200 + (constant));
        double red = -Math.sin(ticktime / 200 + (constant));

        green *= 255 / 2;
        blue *= 255 / 2;
        red *= 255 / 2;

        green += 255 / 2;
        blue += 255 / 2;
        red += 255 / 2;

        ledBuffer.setRGB(i, (int) red, (int) green, (int) blue);
      }
      led.setData(ledBuffer);
    } else if (ledTheme == LEDTheme.BXSCI) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        if (i % 2 == 0) {
          ledBuffer.setLED(i, Color.kGreen);
        } else {
          ledBuffer.setLED(i, Color.kYellow);
        }
      }
      led.setData(ledBuffer);
    }

    // documentation:
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
    // some inspiration by:
    // https://github.com/SciBorgs/ChargedUp-2023/blob/io-rewrite/src/main/java/org/sciborgs1155/robot/subsystems/LED.java
  }

  // @Log.NT
  // public LEDTheme getTheme() {
  //   return ledThemeNow;
  // }

  private String[] ledBufferData = new String[ledBuffer.getLength()];

  public String[] getBufferData() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBufferData[i] =
          ledBuffer.getLED(i).toHexString(); // it is a hex number but it spits it out
    }
    return ledBufferData;
  }

  @Log.NT
  public String getBufferDataString() {
    return ("[" + String.join(",", getBufferData()) + "]");
  }

  public Command setTheme(LEDTheme ledTheme) {
    return run(() -> setLEDTheme(ledTheme));
  }

  @Override
  public void close() throws Exception {
    led.close();
  }

  // @Override
  // public void periodic() {
  //   System.out.println(getBufferDataString());
  // }
}
