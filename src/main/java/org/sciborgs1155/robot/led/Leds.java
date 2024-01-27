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
    IN_INTAKE, // Look at Constants, Orange 100%
    IN_PASSING, // Look at Constants, Grey 100% (but they write it as gray)
    IN_SHOOTER, // Look at Constants, Green 100%
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
      case IN_INTAKE:
        setSolidColor(INTAKE_COLOR); // these colors are defined in constants
        led.setData(ledBuffer);
        break;
      case IN_PASSING:
        setSolidColor(PASSING_COLOR); // these colors are defined in constants
        led.setData(ledBuffer);
        break;
      case IN_SHOOTER:
        setSolidColor(SHOOTER_COLOR); // these colors are defined in constants
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
  public void close() {
    led.close();
  }

  // @Override
  // public void periodic() {
  //   System.out.println(getBufferDataString());
  // }
}
