package org.sciborgs1155.robot.setup;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;

public interface Setup {
  public void createFlex(CANSparkFlex motor, Measure<Current> current);
}
