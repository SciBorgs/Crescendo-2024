package org.sciborgs1155.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LedConstants {
  public static final Color INTAKE_COLOR = Color.kOrange;
  public static final Color PASSING_COLOR = Color.kYellow;
  public static final Color SHOOTER_COLOR = Color.kGreen;
  public static final Color OUTOFRANGE_COLOR = Color.kRed;
  public static final Color NO_BAGEL_COLOR = Color.kBrown;

  public static final int LEDLENGTH = 30; // change to be length of LED strip?

  
  // specifically for raindrop
  public static final int[] shape = {10, 3};
  // number of rows, then LEDs per row, REMEMBER TO CHANGE COLUMN NUMBER TO LEDS PER ROW IN THE SIM
  public static Color[][] grid = new Color[shape[0]][shape[1]];
  public static final Color[] colorpool = {
    Color.kRed, Color.kOrange, Color.kYellow, Color.kGreen, Color.kBlue, Color.kPurple
  };

  
}
