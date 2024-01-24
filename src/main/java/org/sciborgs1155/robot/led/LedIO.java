package org.sciborgs1155.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import monologue.Logged;

public interface LedIO extends Logged, AutoCloseable {
  void setData(AddressableLEDBuffer ledBuffer);
}
