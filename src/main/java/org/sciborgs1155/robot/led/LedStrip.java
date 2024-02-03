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

  // 1 tick = 0.02 seconds (because periodic)   50 ticks = 1 second (minecraft gameticks x2.5 speed)
  static double tick = 0; // needs to be double or else rainbow breaks
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
      ledBuffer.setLED(i, i % 2 == 0 ? color1 : color2);
    }
    return ledBuffer;
  }

  /** "every (every) LEDs, LED should be (color 2). everyting else is (color 1)." */
  private static AddressableLEDBuffer setMovingColor(Color color1, Color color2, int every) {
    final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, (i + tick) % every == 0 ? color2 : color1);
    }
    return ledBuffer;
  }

  // A bunch of methoeds for the LEDThemes below!
  // (the ones that can't be copy paste)

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
    Color[] fireColors = {
      Color.kRed, Color.kOrange, Color.kYellow, Color.kOrangeRed, Color.kOrange
    };
    AddressableLEDBuffer ledBuffer =
        new AddressableLEDBuffer(LEDLENGTH); // the robot is lit! (but it should not burning)
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, fireColors[(int) (Math.floor((i + tick) % 5))]);
    }
    return ledBuffer;
  }

  private static Color[] raindrop = new Color[LEDLENGTH];

  private static AddressableLEDBuffer setRaindrop() {
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
