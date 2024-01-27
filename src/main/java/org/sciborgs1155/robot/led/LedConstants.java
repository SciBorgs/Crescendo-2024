package org.sciborgs1155.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LedConstants {
     public static final Color INTAKE_COLOR = Color.kOrange;
    public static final Color PASSING_COLOR = Color.kGray;
    public static final Color SHOOTER_COLOR = Color.kGreen;
    public static final int LEDLENGTH = 120; // change to be length of LED strip?

    static double ticktime = 0;
    static int temp = 0; 

    // 1 tick = 0.005 seconds    200 ticks = 1 second (minecraft gameticks x20 speed)

    public static void setRainbow(AddressableLEDBuffer ledBuffer){
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

    public static void setSciborgs(AddressableLEDBuffer ledBuffer){
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        if (i % 2 == 0) {
            ledBuffer.setLED(i, Color.kDarkGray);
        } else {
          ledBuffer.setLED(i, Color.kYellow);
        }
      }
    }

    public static void setBXSCIFlash(AddressableLEDBuffer ledBuffer){
      ticktime += 1;
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        if ((i + ticktime) % 5 == 0) {
          ledBuffer.setLED(i, Color.kYellow);
        } else {
          ledBuffer.setLED(i, Color.kGreen);
        }
      }
    }

    public static void setAuto(AddressableLEDBuffer ledBuffer){
      ticktime+=1;
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        temp=(int) (i + ticktime) % 3;
        if (temp == 0) {
          ledBuffer.setLED(i, Color.kYellowGreen);
        } else if (temp == 1){
          ledBuffer.setLED(i, Color.kGreen);
        } else if (temp == 2){
          ledBuffer.setLED(i, Color.kGold);
        }
      }
    }

}