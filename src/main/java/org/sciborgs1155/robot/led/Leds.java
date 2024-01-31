package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.Led.*;
import static org.sciborgs1155.robot.led.LedConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Leds extends SubsystemBase implements Logged, AutoCloseable {
  private final AddressableLED led = new AddressableLED(LEDPORT); // led as a class
  public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);

  public static enum LEDTheme {
    RAINBOW, // RGB Gamer Robot
    SCIBORGS, // Yellow 50%, Dark Grey 50%
    FEMAIDENS, // Yellow 50%, Green 50%
    BXSCIFLASH, // Yellow ??%, Green ??%, moving
    IN_INTAKE_INRANGE, // Look at Constants and Code Below
    IN_PASSING_INRANGE, // Look at Constants and Code Below
    IN_SHOOTER_INRANGE, // Look at Constants and Code Below
    IN_INTAKE_OUTRANGE, // Look at Constants and Code Below
    IN_PASSING_OUTRANGE, // Look at Constants and Code Below
    IN_SHOOTER_OUTRANGE, // Look at Constants and Code Below
    AUTO, // Yellow Green 33%, Green 33%, Gold 33% , moving
    LIT, // Suppose to look like fire
    CHASE, // Looks like those store lights chasing eachother in a loop
    RAINDROP // falling notes thing, random colors drop from the top
  }

  static double ticktime = 0;

  public Leds() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void setLEDTheme(LEDTheme ledTheme) {
    switch (ledTheme) {
      case RAINBOW:
        setRainbow(ledBuffer);
        led.setData(ledBuffer);
        break;
      case SCIBORGS:
        setSciborgs(ledBuffer);
        led.setData(ledBuffer);
        break;
      case FEMAIDENS:
        setFeMaidens(ledBuffer);
        led.setData(ledBuffer);
        break;
      case BXSCIFLASH:
        setBXSCIFlash(ledBuffer);
        led.setData(ledBuffer);
        break;
      case IN_INTAKE_INRANGE:
        // these colors are defined in constants, meant for when the robot is IN shooting range
        setSolidColor(INTAKE_COLOR);
        led.setData(ledBuffer);
        break;
      case IN_PASSING_INRANGE:
        // these colors are defined in constants, meant for when the robot is IN shooting range
        setSolidColor(PASSING_COLOR);
        led.setData(ledBuffer);
        break;
      case IN_SHOOTER_INRANGE:
        // these colors are defined in constants, meant for when the robot is IN shooting range
        setSolidColor(SHOOTER_COLOR);
        led.setData(ledBuffer);
        break;
      case IN_INTAKE_OUTRANGE:
        // these colors are defined in constants, meant for when robot is OUT of shooting range
        setSplitColor(INTAKE_COLOR, OUTOFRANGE_COLOR);
        led.setData(ledBuffer);
        break;
      case IN_PASSING_OUTRANGE:
        // these colors are defined in constants, meant for when robot is OUT of shooting range
        setSplitColor(PASSING_COLOR, OUTOFRANGE_COLOR);
        led.setData(ledBuffer);
        break;
      case IN_SHOOTER_OUTRANGE:
        // these colors are defined in constants, meant for when robot is OUT of shooting range
        setSplitColor(SHOOTER_COLOR, OUTOFRANGE_COLOR);
        led.setData(ledBuffer);
        break;
      case AUTO:
        setAuto(ledBuffer);
        led.setData(ledBuffer);
        break;
      case LIT:
        setFire(ledBuffer);
        led.setData(ledBuffer);
        break;
      case CHASE:
        setChase(ledBuffer);
        led.setData(ledBuffer);
        break;
      case RAINDROP:
        setRaindrop(ledBuffer);
        led.setData(ledBuffer);
        break;
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

  public void setSolidColor(Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
    led.setData(ledBuffer);
  }

  public void setSplitColor(Color color1, Color color2) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        ledBuffer.setLED(i, color1);
      } else {
        ledBuffer.setLED(i, color2);
      }
    }
    led.setData(ledBuffer);
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

  public Command setHalfHalfColor(Color color1, Color color2) {
    return run(() -> setSplitColor(color1, color2));
  }

  @Override
  public void close() {
    led.close();
  }

  // @Override
  // public void periodic() {
  //   System.out.println(getBufferDataString());
  // }
}
