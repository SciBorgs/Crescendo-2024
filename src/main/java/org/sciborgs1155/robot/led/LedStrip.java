package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.Led.LED_PORT;
import static org.sciborgs1155.robot.led.LedConstants.LEDLENGTH;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Function;
import java.util.function.Supplier;
import monologue.Logged;

public class LedStrip extends SubsystemBase implements Logged, AutoCloseable {
  private final AddressableLED led = new AddressableLED(LED_PORT); // led as a class

  // NOTE: THERE CAN ONLY BE ONE ADDRESABLELED (because roborio)

  public static enum LEDTheme {
    BXSCIFLASH(() -> movingColor(Color.kGreen, Color.kYellow, 5)), // Yellow ??%, Green ??%, moving
    CANSHOOT(() -> movingColor(Color.kGreen, Color.kLime, 5)),
    FIRE(LedStrip::fire), // Suppose to look like fire
    RAINBOW(LedStrip::rainbow), // RGB Gamer Robot
    SCIBORGS(() -> movingColor(Color.kYellow, Color.kYellow, 4)), // Yellow 75%, Yellow 25%
    FEMAIDENS(() -> alternatingColor(Color.kPurple, Color.kLime)), // Yellow 50%, Green 50%
    ALLIANCE(() -> allianceColor()),
    AUTO(
        () ->
            movingColor(
                Color.kLawnGreen,
                Color.kCrimson,
                3)), // Yellow Green 33%, Green 33%, Gold 33% , moving
    CHASE(
        () ->
            movingColor(
                Color.kDeepSkyBlue,
                Color.kCrimson,
                5)), // Looks like those store lights chasing eachother in a loop
    RAINDROP(LedStrip::raindrop), // falling notes thing, random colors drop from the top
    TEST(
        () ->
            alternatingColor(
                Color.kYellow,
                Color.kDarkGray)), // theme used for tests (DO NOT CHANGE) Yellow 50%, Dark Grey 25%
    NONE(LedStrip::nothing); // does nothing

    public final Supplier<AddressableLEDBuffer> ledBuffer;

    private LEDTheme(Supplier<AddressableLEDBuffer> ledBuffer) {
      this.ledBuffer = ledBuffer;
    }
  }

  // 1 tick = 0.02 seconds (because periodic)   50 ticks = 1 second
  static double tick = 0; // needs to be double or else rainbow breaks
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

  public static AddressableLEDBuffer gen(Function<Integer, Color> f) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDLENGTH);
    for (int i = 0; i < LEDLENGTH; i++) {
      buffer.setLED(i, f.apply(i));
    }
    return buffer;
  }

  private static AddressableLEDBuffer alternatingColor(Color color1, Color color2) {
    return gen(i -> i % 2 == 0 ? color1 : color2);
  }

  /** "every (interval) LEDs, LED should be (color 2). everyting else is (color 1)." */
  private static AddressableLEDBuffer movingColor(Color color1, Color color2, int interval) {
    return gen(i -> (i + tick) % interval == 0 ? color2 : color1);
  }

  private static AddressableLEDBuffer allianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        return gen(i -> (i + tick) % 2 == 0 ? Color.kBlue : Color.kDarkCyan);
      }
      if (alliance.get() == DriverStation.Alliance.Red) {
        return gen(i -> (i + tick) % 2 == 0 ? Color.kRed : Color.kCrimson);
      }
    }
    return gen(i -> (i + tick) % 2 == 0 ? Color.kOrange : Color.kYellow);
  }

  // A bunch of methoeds for the LEDThemes below!
  // (the ones that can't be copy paste)

  private static AddressableLEDBuffer rainbow() {
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);
    final double scalar = 255 / 2;
    return gen(
        i -> {
          final double theta = tick / 10 + i / (ledBuffer.getLength() * (Math.PI / 2));
          return new Color(
              (int) ((-Math.sin(theta) + 1) * scalar),
              (int) ((Math.sin(theta) + 1) * scalar),
              (int) ((Math.cos(theta) + 1) * scalar));
        });
  }

  private static AddressableLEDBuffer fire() {
    Color[] fireColors = {
      Color.kRed, Color.kOrange, Color.kYellow, Color.kOrangeRed, Color.kOrange
    };
    return gen(i -> fireColors[(int) (Math.floor((i + tick) % 5))]);
  }

  private static Color[] raindrop = new Color[LEDLENGTH];

  private static AddressableLEDBuffer raindrop() {
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

    return gen(i -> raindrop[i] == null ? Color.kBlack : raindrop[i]);
  }

  private static AddressableLEDBuffer nothing() {
    return new AddressableLEDBuffer(LEDLENGTH);
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
