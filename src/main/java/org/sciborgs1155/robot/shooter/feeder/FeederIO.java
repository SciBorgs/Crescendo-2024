package org.sciborgs1155.robot.shooter.feeder;

import edu.wpi.first.units.*;
import edu.wpi.first.units.Measure;

public interface FeederIO extends AutoCloseable {
  public void set(Measure<Voltage> voltage);

  public Measure<Voltage> getVoltage();
}
