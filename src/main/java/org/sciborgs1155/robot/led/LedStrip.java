package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.Led.*;
import static org.sciborgs1155.robot.led.LedConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import monologue.Logged;

public class LedStrip extends SubsystemBase implements Logged, AutoCloseable {
  private final AddressableLED led = new AddressableLED(LEDPORT); // led as a class

  public static enum LEDTheme {
    RAINBOW(LedStrip::setRainbow), // RGB Gamer Robot
    SCIBORGS(
        () -> setAlternatingColor(Color.kDarkGray, Color.kYellow)), // Yellow 50%, Dark Grey 50%
    FEMAIDENS(() -> setAlternatingColor(Color.kPurple, Color.kLime)), // Yellow 50%, Green 50%
    BXSCIFLASH(
        () -> setMovingColor(Color.kGreen, Color.kYellow, 5)), // Yellow ??%, Green ??%, moving
    GREEN(() -> setSolidColor(Color.kGreen)),
    RED(() -> setSolidColor(Color.kRed)),
    AUTO(
        () ->
            setMovingColor(
                Color.kBlack, Color.kYellow, 3)), // Yellow Green 33%, Green 33%, Gold 33% , moving
    LIT(LedStrip::setFire), // Suppose to look like fire
    CHASE(
        () ->
            setMovingColor(
                Color.kDeepSkyBlue,
                Color.kDarkMagenta,
                6)), // Looks like those store lights chasing eachother in a loop
    RAINDROP(LedStrip::setRaindrop), // falling notes thing, random colors drop from the top
    NONE(LedStrip::nothing); // does nothing

    public final Supplier<AddressableLEDBuffer> ledBuffer;

    private LEDTheme(Supplier<AddressableLEDBuffer> ledBuffer) {
      this.ledBuffer = ledBuffer;
    }
  }

  // 1 tick = 0.005 seconds    200 ticks = 1 second (minecraft gameticks x20 speed)
  static double tick = 0;
  static double temp = 0;
  static boolean inrange = false;
  static LEDTheme lastTheme = LEDTheme.NONE;
  public static final Color[] colorpool = {
    Color.kRed, Color.kOrange, Color.kYellow, Color.kGreen, Color.kBlue, Color.kPurple
  };

  public LedStrip() {
    led.setLength(LEDLENGTH);
    led.setData(nothing());
    led.start();
  }

  /** New Command to set LedTheme LEDs (look at enum) */
  public Command setLEDTheme(LEDTheme ledTheme) {
    return run(
        () -> {
          led.setData(ledTheme.ledBuffer.get());
        });
  }

  // documentation:
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
  // some inspiration by:
  // https://github.com/SciBorgs/ChargedUp-2023/blob/io-rewrite/src/main/java/org/sciborgs1155/robot/subsystems/LED.java

  public static String getBufferDataString(AddressableLEDBuffer ledBuffer) {
    String[] ledBufferData = new String[ledBuffer.getLength()];
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBufferData[i] =
          ledBuffer.getLED(i).toHexString(); // it is a hex number but it spits it out
    }
    return ("[" + String.join(",", ledBufferData) + "]");
  }

  @Override
  public void close() {
    led.close();
  }

  // @Override
  // public void periodic() {
  //   System.out.println(getBufferDataString());
  // }

  private static AddressableLEDBuffer setSolidColor(Color color) {
    final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
    return ledBuffer;
  }

  private static AddressableLEDBuffer setAlternatingColor(Color color1, Color color2) {
    final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        ledBuffer.setLED(i, color1);
      } else {
        ledBuffer.setLED(i, color2);
      }
    }
    return ledBuffer;
  }

  /** "every (every) LEDs, LED should be (color 2). everyting else is (color 1)." */
  private static AddressableLEDBuffer setMovingColor(Color color1, Color color2, int every) {
    final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if ((i + tick) % every == 0) {
        ledBuffer.setLED(i, color2);
      } else {
        ledBuffer.setLED(i, color1);
      }
    }
    return ledBuffer;
  }

  // A bunch of methoeds for the LEDThemes below!
  // (the ones that can't be copy paste) (some exceptions)

  private static AddressableLEDBuffer setRainbow() {
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final double constant = i / (ledBuffer.getLength() * (Math.PI / 2));
      double green = Math.sin(((tick * 20) / 200) + (constant));
      double blue = Math.cos(((tick * 20) / 200) + (constant));
      double red = -Math.sin(((tick * 20) / 200) + (constant));
      green *= 255 / 2;
      blue *= 255 / 2;
      red *= 255 / 2;
      green += 255 / 2;
      blue += 255 / 2;
      red += 255 / 2;
      ledBuffer.setRGB(i, (int) red, (int) green, (int) blue);
    }
    return ledBuffer;
  }

  private static AddressableLEDBuffer setFire() {
    AddressableLEDBuffer ledBuffer =
        new AddressableLEDBuffer(LEDLENGTH); // the robot is lit! (but it should not burning)
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      temp = (i + tick) % 5;
      if (temp < 1) {
        ledBuffer.setLED(i, Color.kRed);
      } else if (temp < 2) {
        ledBuffer.setLED(i, Color.kOrange);
      } else if (temp < 3) {
        ledBuffer.setLED(i, Color.kYellow);
      } else if (temp < 4) {
        ledBuffer.setLED(i, Color.kOrangeRed);
      } else if (temp < 5) {
        ledBuffer.setLED(i, Color.kOrange);
      }
    }
    return ledBuffer;
  }

  private static Color[] raindrop = new Color[LEDLENGTH];

  private static AddressableLEDBuffer setRaindrop() {
    // start on 2:26 PM speedrun, end speedrun on 4:51 PM
    // (i spent an hour trying to fix something and learned that java lists in lists suck!)
    // code is now changed, the fix was that java lists of lists apparently arent a list of LISTS
    // why can we destroy faster than we create

    // REQUIRES FINAL GRID SHAPE DIMENSIONS, ASSUMES FORTH AND FORTH ONLY LAYERING!
    // (back and forth would be easy to do, edit some parts, add reverse list row code)
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);

    if (Math.round(Math.random() * 2) == 0) {
      raindrop[0] = Color.kBlack;
    } else {
      if (Math.round(Math.random()) == 0) {
        raindrop[0] = colorpool[(int) (Math.round(Math.random() * (colorpool.length - 1)))];
      }
    }

    for (int i = raindrop.length - 1; i > 0; i -= 1) {
      raindrop[i] = raindrop[i - 1];
    }

    for (int i = 0; i < raindrop.length; i++) {
      if (raindrop[i] == null) {
        ledBuffer.setLED(i, Color.kBlack);
      } else {
        ledBuffer.setLED(i, raindrop[i]);
      }
    }
    return ledBuffer;
  }

  private static AddressableLEDBuffer nothing() {
    return new AddressableLEDBuffer(LEDLENGTH);
  }

  @Override
  public void periodic() {
    tick += 1;
  }
}
