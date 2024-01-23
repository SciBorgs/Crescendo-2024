package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.Led.*;

import static org.sciborgs1155.robot.Constants.Led.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RealLed implements LedIO {
    AddressableLED led;
    static double time = 0;
    
    public RealLed(){
        led = new AddressableLED(LEDPORT);
        led.setLength(LEDLENGTH);
    }

    @Override
    public void setData(AddressableLEDBuffer ledBuffer) {
        led.setData(ledBuffer);
    }


}
