package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Constants.Led.*;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Leds extends SubsystemBase implements Logged, AutoCloseable {
  private final LedIO led = Robot.isReal() ? new RealLed() : new SimLed(); // led as a class
  public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);

  public static enum LEDTheme {
    RAINBOW, // RGB Gamer Robot
    BXSCI, // Yellow 50%, Green 50%
    BXSCIFLASH, // Yellow ??%, Green ??%, Moving,
    // (All Yellow moves in one direction, yellow every few LEDs, green rest)
    IN_INTAKE, // Look at Constants, Orange 100%
    IN_PASSING, // Look at Constants, Grey 100% (but they write it as gray)
    IN_SHOOTER, // Look at Constants, Green 100%
    AUTO, // Yellow 50%, Black 50%
    ERROR, // Red 100%
    EXPLODE // Red 100%
  }

  static double ticktime = 0;
  private LEDTheme ledThemeNow = LEDTheme.AUTO;

  public Leds() {
    setLEDTheme(
        LEDTheme
            .AUTO); // put default on initialization here (only run once though, so moving themes
    // will not move)
  }

  public void setLEDTheme(LEDTheme ledTheme) {
    ledThemeNow = ledTheme;
    if (ledTheme == LEDTheme.RAINBOW) {
      ticktime +=
          15; // 1 tick = 0.005 seconds    200 ticks = 1 second (minecraft gameticks x20 speed)
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        final double constant = i / (ledBuffer.getLength() * (Math.PI / 2));
        double green = Math.sin((ticktime / 200) + (constant));
        double blue = Math.cos((ticktime / 200) + (constant));
        double red = -Math.sin((ticktime / 200) + (constant));
        green *= 255 / 2;
        blue *= 255 / 2;
        red *= 255 / 2;
        green += 255 / 2;
        blue += 255 / 2;
        red += 255 / 2;
        ledBuffer.setRGB(i, (int) red, (int) green, (int) blue);
      }
    } else if (ledTheme == LEDTheme.BXSCI) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        if (i % 2 == 0) {
          ledBuffer.setLED(i, Color.kGreen);
        } else {
          ledBuffer.setLED(i, Color.kYellow);
        }
      }
    } else if (ledTheme == LEDTheme.BXSCIFLASH) {
      ticktime += 1;
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        if (((i + ticktime) / 3) % 5 == 0) {
          ledBuffer.setLED(i, Color.kYellow);
        } else {
          ledBuffer.setLED(i, Color.kGreen);
        }
      }
    } else if (ledTheme == LEDTheme.IN_INTAKE) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, INTAKE_COLOR);
      }
    } else if (ledTheme == LEDTheme.IN_PASSING) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, PASSING_COLOR);
      }
    } else if (ledTheme == LEDTheme.IN_SHOOTER) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, SHOOTER_COLOR);
      }
    } else if (ledTheme == LEDTheme.AUTO) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        if (i % 2 == 0) {
          ledBuffer.setLED(i, Color.kYellow);
        } else {
          ledBuffer.setLED(i, Color.kBlack);
        }
      }
    } else if ((ledTheme == LEDTheme.EXPLODE) || (ledTheme == LEDTheme.ERROR)) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, Color.kRed);
      }
    }

    led.setData(ledBuffer);

    // documentation:
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
    // some inspiration by:
    // https://github.com/SciBorgs/ChargedUp-2023/blob/io-rewrite/src/main/java/org/sciborgs1155/robot/subsystems/LED.java
  }

  public void setSolidColor(Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
    led.setData(ledBuffer);
  }

  @Log.NT
  public LEDTheme getTheme() {
    return ledThemeNow;
  }

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

  public Command setColor(Color color) {
    return run(() -> setSolidColor(color));
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
