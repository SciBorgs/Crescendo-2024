package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.Led.LED_PORT;
import static org.sciborgs1155.robot.led.LedConstants.LED_LENGTH;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Function;
import monologue.Logged;

public class LedStrip extends SubsystemBase implements Logged, AutoCloseable {
  private final AddressableLED led = new AddressableLED(LED_PORT);

  // NOTE: THERE CAN ONLY BE ONE ADDRESABLELED (because roborio)

  // Current LEDThemes:
  // Autonoumous - Rainbow
  // Default (not doing anything speical) - Fire
  // Robot Has A Note - Chase
  // Note Has Left The Robot - Raindrop for 0.5s

  static double tick = 0; // needs to be double or else rainbow breaks
  public static final Color[] colorpool = {
    Color.kRed, Color.kOrange, Color.kYellow, Color.kGreen, Color.kBlue, Color.kPurple
  };

  public LedStrip() {
    led.setLength(LED_LENGTH);
    led.setData(new AddressableLEDBuffer(LED_LENGTH));
    led.start();
  }

  public Command set(AddressableLEDBuffer addressableLEDBuffer) {
    return run(() -> led.setData(addressableLEDBuffer));
  }

  public Command orange() {
    return set(solidColor(Color.kOrange));
  }

  public Command blue() {
    return set(solidColor(Color.kDeepSkyBlue));
  }

  public Command bxSciFlash() {
    return set(movingColor(Color.kGreen, Color.kYellow, 5));
  }

  public Command fire() {
    Color[] fireColors = {
      Color.kRed, Color.kOrange, Color.kYellow, Color.kOrangeRed, Color.kOrange
    };
    return set(gen(i -> fireColors[(int) (Math.floor((i + tick) % 5))]));
  }

  public Command rainbow() {
    final double scalar = 255 / 2;
    return set(
        gen(
            i -> {
              final double theta = tick / 10 + i / (LED_LENGTH * (Math.PI / 2));
              return new Color(
                  (int) ((-Math.sin(theta) + 1) * scalar),
                  (int) ((Math.sin(theta) + 1) * scalar),
                  (int) ((Math.cos(theta) + 1) * scalar));
            }));
  }

  public Command sciborgs() {
    return set(movingColor(Color.kYellow, Color.kYellow, 4));
  }

  public Command femaidens() {
    return set(alternatingColor(Color.kPurple, Color.kLime));
  }

  public Command alliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        return set(gen(i -> (i + tick) % 2 == 0 ? Color.kBlue : Color.kDarkCyan));
      }
      if (alliance.get() == DriverStation.Alliance.Red) {
        return set(gen(i -> (i + tick) % 2 == 0 ? Color.kRed : Color.kCrimson));
      }
    }
    return set(gen(i -> (i + tick) % 2 == 0 ? Color.kOrange : Color.kYellow));
  }

  public Command chase() {
    return set(movingColor(Color.kDeepSkyBlue, Color.kCrimson, 5));
  }

  private static Color[] raindrop = new Color[LED_LENGTH];

  public Command raindrop() {
    if (Math.round(Math.random()) == 0) {
      raindrop[0] = Color.kBlack;
    } else {
      if (Math.round(Math.random()) == 0) {
        raindrop[0] = colorpool[(int) (Math.round(Math.random() * (colorpool.length - 1)))];
      }
    }

    for (int i = raindrop.length - 1; i > 0; i -= 1) {
      raindrop[i] = raindrop[i - 1];
    }

    return set(gen(i -> raindrop[i] == null ? Color.kBlack : raindrop[i]));
  }

  // wont work now
  // public Command test() {
  //   return set(alternatingColor(Color.kYellow, Color.kDarkGray));
  // }

  public Command none() {
    return set(new AddressableLEDBuffer(LED_LENGTH));
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

  public static AddressableLEDBuffer gen(Function<Integer, Color> f) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_LENGTH);
    for (int i = 0; i < LED_LENGTH; i++) {
      buffer.setLED(i, f.apply(i));
    }
    return buffer;
  }

  private static AddressableLEDBuffer alternatingColor(Color color1, Color color2) {
    return gen(i -> i % 2 == 0 ? color1 : color2);
  }

  private static AddressableLEDBuffer solidColor(Color color1) {
    return gen(i -> color1);
  }

  /** "every (interval) LEDs, LED should be (color 2). everyting else is (color 1)." */
  private static AddressableLEDBuffer movingColor(Color color1, Color color2, int interval) {
    return gen(i -> (i + tick) % interval == 0 ? color2 : color1);
  }

  @Override
  public void periodic() {
    tick += 1;
  }

  @Override
  public void close() {
    led.close();
  }
}
