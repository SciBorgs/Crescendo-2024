package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Constants.Led.*;
import static org.sciborgs1155.robot.Ports.Led.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class SimLed implements LedIO {
  public static AddressableLED led = new AddressableLED(LEDPORT);
  private static final AddressableLEDSim ledsim = new AddressableLEDSim(led);

  public SimLed() {
    led.setLength(LEDLENGTH);
  }

  @Override
  public void setData(AddressableLEDBuffer ledBuffer) {
    led.setData(ledBuffer);
  }

  public byte[] getData() {
    return ledsim.getData();
  }

  // work on this
  // documentation:
  // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/AddressableLEDSim.html

  @Override
  public void close() throws Exception {
    led.close();
    ledsim.resetData();
  }
}
