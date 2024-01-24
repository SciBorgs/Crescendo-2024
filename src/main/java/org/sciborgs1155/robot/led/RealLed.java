package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Constants.Led.*;
import static org.sciborgs1155.robot.Ports.Led.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RealLed implements LedIO {
  public static AddressableLED led = new AddressableLED(LEDPORT);

  public RealLed() {
    led.setLength(LEDLENGTH);
  }

  @Override
  public void setData(AddressableLEDBuffer ledBuffer) {
    led.setData(ledBuffer);
  }

  @Override
  public void close() throws Exception {
    led.close();
  }
}
