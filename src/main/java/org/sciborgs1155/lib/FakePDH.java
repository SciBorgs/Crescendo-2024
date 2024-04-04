package org.sciborgs1155.lib;

import com.revrobotics.CANSparkBase;
import java.util.ArrayList;
import java.util.List;

/**
 * Our actual PDH can't connect to CAN, so this is a workaround to (poorly) estimate our total used
 * current based on the stator currents of all our motors.
 */
public class FakePDH {
  private static final List<CANSparkBase> sparks = new ArrayList<>();
  private static double current;

  public static void register(CANSparkBase spark) {
    sparks.add(spark);
  }

  public static double update() {
    current = sparks.stream().mapToDouble(CANSparkBase::getOutputCurrent).sum();
    return current;
  }

  public static double current() {
    return current;
  }
}
