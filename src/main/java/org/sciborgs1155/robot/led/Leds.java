package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.led.LedConstants.*;
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
    SCIBORGS, // Yellow 50%, Dark Grey 50%
    BXSCIFLASH, // Yellow ??%, Green ??% but moving around
    IN_INTAKE, // Look at Constants, Orange 100%
    IN_PASSING, // Look at Constants, Grey 100% (but they write it as gray)
    IN_SHOOTER, // Look at Constants, Green 100%
    AUTO, // Yellow 50%, Black 50%
  }

  static double ticktime = 0;

  public Leds() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void setLEDTheme(LEDTheme ledTheme) {
    switch (ledTheme){
      case RAINBOW: 
        setRainbow(ledBuffer); 
        led.setData(ledBuffer);
        break;
      case SCIBORGS: 
        setSciborgs(ledBuffer); 
        break;
      case BXSCIFLASH:
        setBXSCIFlash(ledBuffer);
        break;
      case IN_INTAKE:
        setSolidColor(INTAKE_COLOR); //these colors are defined in constants 
        break;
      case IN_PASSING:
        setSolidColor(PASSING_COLOR); //these colors are defined in constants 
        break;
      case IN_SHOOTER:
        setSolidColor(SHOOTER_COLOR); //these colors are defined in constants 
        break;
      case AUTO:
        setAuto(ledBuffer);
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
  public void close() throws Exception {
    led.close();
  }

  // @Override
  // public void periodic() {
  //   System.out.println(getBufferDataString());
  // }
}
