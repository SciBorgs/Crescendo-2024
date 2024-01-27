package org.sciborgs1155.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LedConstants {
  public static final Color INTAKE_COLOR = Color.kOrange;
  public static final Color PASSING_COLOR = Color.kGray;
  public static final Color SHOOTER_COLOR = Color.kGreen;
  public static final int LEDLENGTH = 120; // change to be length of LED strip?

  static double ticktime = 0;
  static double temp = 0;

  // 1 tick = 0.005 seconds    200 ticks = 1 second (minecraft gameticks x20 speed)

  // specifically for raindrop
  private static final int[] shape = {12, 10};
  // number of rows, then LEDs per row, REMEMBER TO CHANGE COLUMN NUMBER TO LEDS PER ROW IN THE SIM
  static Color[][] grid = new Color[shape[0]][shape[1]];
  private static final Color[] colorpool = {
    Color.kRed, Color.kOrange, Color.kYellow, Color.kGreen, Color.kBlue, Color.kPurple
  };

  public static void setRainbow(AddressableLEDBuffer ledBuffer) {
    ticktime += 13;
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

  public static void setSciborgs(AddressableLEDBuffer ledBuffer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        ledBuffer.setLED(i, Color.kDarkGray);
      } else {
        ledBuffer.setLED(i, Color.kYellow);
      }
    }
  }

  public static void setFeMaidens(AddressableLEDBuffer ledBuffer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        ledBuffer.setLED(i, Color.kPurple);
      } else {
        ledBuffer.setLED(i, Color.kGreen);
      }
    }
  }

  public static void setBXSCIFlash(AddressableLEDBuffer ledBuffer) {
    ticktime += 1;
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if ((i + ticktime) % 5 == 0) {
        ledBuffer.setLED(i, Color.kYellow);
      } else {
        ledBuffer.setLED(i, Color.kGreen);
      }
    }
  }

  public static void setAuto(AddressableLEDBuffer ledBuffer) {
    ticktime += 0.1;
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      temp = (i + ticktime) % 3;
      if (temp < 1) {
        ledBuffer.setLED(i, Color.kYellowGreen);
      } else if (temp < 2) {
        ledBuffer.setLED(i, Color.kGreen);
      } else if (temp < 3) {
        ledBuffer.setLED(i, Color.kGold);
      }
    }
  }

  public static void setFire(AddressableLEDBuffer ledBuffer) {
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

  public static void setChase(AddressableLEDBuffer ledBuffer) {
    ticktime += 0.3;
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      temp = (i + ticktime) % 6;
      if (temp < 1) {
        ledBuffer.setLED(i, Color.kYellow);
      } else if (temp > 3 && temp < 4) {
        ledBuffer.setLED(i, Color.kLimeGreen);
      } else {
        ledBuffer.setLED(i, Color.kGreen);
      }
    }
  }

  public static void setRaindrop(AddressableLEDBuffer ledBuffer) {
    // start on 2:26 PM speedrun, end speedrun on 4:51 PM
    // (i spent an hour trying to fix something and learned that java lists in lists suck!)
    // REQUIRES FINAL GRID SHAPE DIMENSIONS, ASSUMES FORTH AND FORTH ONLY LAYERING!
    // (back and forth would be easy to do, edit some parts, add reverse list row code)
    ticktime +=0.1;

    for (int i = 0; i < shape[1]; i++) {
      grid[0][i] = Color.kBlack;
    }

    grid[0][(int) (Math.round(Math.random() * (shape[1] - 1)))] =
        colorpool[(int) (Math.round(Math.random() * (colorpool.length - 1)))];

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
}
