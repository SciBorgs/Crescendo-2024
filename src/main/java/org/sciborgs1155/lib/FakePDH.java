package org.sciborgs1155.lib;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;

/**
 * Our old PDH couldn't connect to CAN, so this was a workaround to (poorly) estimate our total used
 * current based on the stator currents of all our motors with the formula:
 *
 * <p>supply current = battery voltage * applied output * stator current
 */
public class FakePDH {
  private static final List<CANSparkBase> sparks = new ArrayList<>();
  private static double statorCurrent;
  private static double supplyCurrent;

  public static void register(CANSparkBase spark) {
    sparks.add(spark);
  }

  public static double update() {
    double voltage = RobotController.getBatteryVoltage();
    double output = sparks.stream().mapToDouble(CANSparkBase::getAppliedOutput).sum();
    statorCurrent =
        sparks.stream().mapToDouble(CANSparkBase::getOutputCurrent).map(Math::abs).sum();
    supplyCurrent = voltage * output * statorCurrent;
    return supplyCurrent;
  }

  public static double supplyCurrent() {
    return supplyCurrent;
  }

  public static double statorCurrent() {
    return statorCurrent;
  }
}
