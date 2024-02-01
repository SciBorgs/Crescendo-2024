package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.Led.*;
import static org.sciborgs1155.robot.led.LedConstants.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Leds extends SubsystemBase implements Logged, AutoCloseable {
  private final static AddressableLED led = new AddressableLED(LEDPORT); // led as a class
  public final static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH);

  public static enum LEDTheme {
    RAINBOW(Leds::setRainbow), // RGB Gamer Robot
    SCIBORGS(Leds::setSciborgs), // Yellow 50%, Dark Grey 50%
    FEMAIDENS(Leds::setFeMaidens), // Yellow 50%, Green 50%
    BXSCIFLASH(Leds::setBXSCIFlash), // Yellow ??%, Green ??%, moving
    NO_BAGEL_INRANGE((buffer) -> setSolidColor(led, buffer, NO_BAGEL_COLOR)), // Look at Constatnts and Code Below
    NO_BAGEL_OUTRANGE((buffer) -> setSplitColor(led, buffer, NO_BAGEL_COLOR, OUTOFRANGE_COLOR)), // Look at Constatnts and Code Below
    IN_INTAKE_INRANGE((buffer) -> setSolidColor(led, buffer, INTAKE_COLOR)), // Look at Constants and Code Below
    IN_INTAKE_OUTRANGE((buffer) -> setSplitColor(led, buffer, INTAKE_COLOR, OUTOFRANGE_COLOR)), // Look at Constants and Code Below
    IN_PASSING_INRANGE((buffer) -> setSolidColor(led, buffer, PASSING_COLOR)), // Look at Constants and Code Below
    IN_PASSING_OUTRANGE((buffer) -> setSplitColor(led, buffer, PASSING_COLOR, OUTOFRANGE_COLOR)), // Look at Constants and Code Below
    IN_SHOOTER_INRANGE((buffer) -> setSolidColor(led, buffer, SHOOTER_COLOR)), // Look at Constants and Code Below
    IN_SHOOTER_OUTRANGE((buffer) -> setSplitColor(led, buffer, SHOOTER_COLOR, OUTOFRANGE_COLOR)), // Look at Constants and Code Below
    AUTO(Leds::setAuto), // Yellow Green 33%, Green 33%, Gold 33% , moving
    LIT(Leds::setFire), // Suppose to look like fire
    CHASE(Leds::setChase), // Looks like those store lights chasing eachother in a loop
    RAINDROP(Leds::setRaindrop), // falling notes thing, random colors drop from the top
    NONE(Leds::nothing); // does nothing

    public final Consumer<AddressableLEDBuffer> setLEDs;
    private LEDTheme(Consumer<AddressableLEDBuffer> setLEDs) {
      this.setLEDs = setLEDs;
    }
  }


  
  // 1 tick = 0.005 seconds    200 ticks = 1 second (minecraft gameticks x20 speed)
  static double ticktime = 0;
  static double temp = 0;
  static boolean inrange = false;
  static LEDTheme lastTheme = LEDTheme.NONE;

  public Leds() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public Command setLEDTheme(LEDTheme ledTheme) {
    return run(() -> {
      ledTheme.setLEDs.accept(ledBuffer);
      led.setData(ledBuffer);
    });
  }
  // documentation:
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
  // some inspiration by:
  // https://github.com/SciBorgs/ChargedUp-2023/blob/io-rewrite/src/main/java/org/sciborgs1155/robot/subsystems/LED.java

  // @Log.NT
  // public LEDTheme getTheme() {
  //   return ledThemeNow;
  // }

  public static void setSolidColor(AddressableLED led, AddressableLEDBuffer ledBuffer, Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
    led.setData(ledBuffer);
  }

  public static void setSplitColor(AddressableLED led, AddressableLEDBuffer ledBuffer, Color color1, Color color2) {
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

  /** Command to set LedTheme LEDs (look at enum) */
  public Command setTheme(LEDTheme ledTheme) {
    return run(() -> setLEDTheme(ledTheme));
  }

  /** Command to setSolidColor LEDs */
  public Command setColor(Color color) {
    return run(() -> setSolidColor(color));
  }

  /** Command to setHalfHalfColor LEDs (50%,50%) */
  public Command setHalfHalfColor(Color color1, Color color2) {
    return run(() -> setSplitColor(color1, color2));
  }

  /**
   * COMMAND TO UPDATE OUT OF RANGE BASED ON SHOOTABILITY BOOLEAN PASSED IN If last LEDTheme is
   * about the bagel/note, theme will be updated to have a 50% OUT OF RANGE color on top of the
   * bagel/note related LEDTheme
   */
  public Command setBasedShootable(boolean shootable) {
    return run(() -> setColorBasedOnShootability(shootable));
  }

  @Override
  public void close() {
    led.close();
  }

  // @Override
  // public void periodic() {
  //   System.out.println(getBufferDataString());
  // }


  private static void setRainbow(AddressableLEDBuffer ledBuffer) {
    ticktime += 20;
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
  }

  private static void setSciborgs(AddressableLEDBuffer ledBuffer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        ledBuffer.setLED(i, Color.kDarkGray);
      } else {
        ledBuffer.setLED(i, Color.kYellow);
      }
    }
  }

  private static void setFeMaidens(AddressableLEDBuffer ledBuffer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        ledBuffer.setLED(i, Color.kPurple);
      } else {
        ledBuffer.setLED(i, Color.kLime);
      }
    }
  }

  private static void setBXSCIFlash(AddressableLEDBuffer ledBuffer) {
    ticktime += 1;
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if ((i + ticktime) % 5 == 0) {
        ledBuffer.setLED(i, Color.kYellow);
      } else {
        ledBuffer.setLED(i, Color.kGreen);
      }
    }
  }

  private static void setAuto(AddressableLEDBuffer ledBuffer) {
    ticktime += 0.3;
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      temp = (i + ticktime) % 3;
      if (temp < 1) {
        ledBuffer.setLED(i, Color.kGold);
      } else if (temp < 2.4) {
        ledBuffer.setLED(i, Color.kBlack);
      }
      //  else if (temp < 3) {
      //   ledBuffer.setLED(i, Color.kBlanchedAlmond);
      // }
    }
  }

  private static void setFire(AddressableLEDBuffer ledBuffer) {
    ticktime += 0.3; // the robot is lit! (but it should not burning)
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      temp = (i + ticktime) % 5;
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
  }

  private static void setChase(AddressableLEDBuffer ledBuffer) {
    ticktime += 0.3;
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      temp = (i + ticktime) % 6;
      if (temp < 2) {
        ledBuffer.setLED(i, Color.kDarkMagenta);
      } else {
        ledBuffer.setLED(i, Color.kDeepSkyBlue);
      }
      // else {
      //   ledBuffer.setLED(i, Color.kDeepSkyBlue);
      // }
    }
  }

  private static void setRaindrop(AddressableLEDBuffer ledBuffer) {
    // start on 2:26 PM speedrun, end speedrun on 4:51 PM
    // (i spent an hour trying to fix something and learned that java lists in lists suck!)
    // REQUIRES FINAL GRID SHAPE DIMENSIONS, ASSUMES FORTH AND FORTH ONLY LAYERING!
    // (back and forth would be easy to do, edit some parts, add reverse list row code)
    ticktime += 0.3;

    if (Math.round(Math.random()) == 0) {
      for (int i = 0; i < shape[1]; i++) {
        grid[0][i] = Color.kBlack;
      }
    } else {
      grid[0][(int) (Math.round(Math.random() * (shape[1] - 1)))] =
          colorpool[(int) (Math.round(Math.random() * (colorpool.length - 1)))];
    }

    for (int i = shape[0] - 1; i > 0; i -= 1) {
      for (int ie = 0; ie < shape[1]; ie += 1) {
        // THIS INSIDE FOR LOOP COST ME AN HOUR
        grid[i][ie] = grid[i - 1][ie];
      }
    }

    for (int i = 0; i < shape[0]; i++) {
      for (int ie = 0; ie < shape[1]; ie++) {
        if (grid[i][ie] == null && grid[i][ie] != Color.kBlack) {
          grid[i][ie] = Color.kBlack;
        }
        ledBuffer.setLED(i * shape[1] + ie, grid[i][ie]);
      }
    }
  }

  private static void nothing(AddressableLEDBuffer ledBuffer){}


}


