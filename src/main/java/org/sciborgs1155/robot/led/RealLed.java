package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.Led.*;

import org.sciborgs1155.robot.led.Led.LEDTheme;

import static org.sciborgs1155.robot.Constants.Led.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class RealLed implements LedIO {
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;
    static double time = 0;
    
    public RealLed(){
        led = new AddressableLED(LEDPORT);
        led.setLength(ledBuffer.getLength());
    }

    @Override
    public void setTheme(LEDTheme ledTheme){
        if (ledTheme == LEDTheme.RAINBOW) {
            time += .005;
            for (int i = 0; i < ledBuffer.getLength(); i++) {

                final double constant = i / (ledBuffer.getLength() * (Math.PI / 2));
                double green = Math.sin(time + (constant));
                double blue = Math.cos(time + (constant));
                double red = -Math.sin(time + (constant));

                green *= 255 / 2;
                blue *= 255 / 2;
                red *= 255 / 2;

                green += 255 / 2;
                blue += 255 / 2;
                red += 255 / 2;

                ledBuffer.setRGB(i, (int) red, (int) green, (int) blue);
                led.setData(ledBuffer);
            }
        } else if (ledTheme == LEDTheme.BXSCI) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                if (i% 2 ==0){ledBuffer.setLED(i, Color.kGreen);} 
                    else {ledBuffer.setLED(i, Color.kYellow);}
            }
        led.setData(ledBuffer);
        } else if (ledTheme == LEDTheme.IN_INTAKE) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, INTAKE_COLOR);
            }
        } else if (ledTheme == LEDTheme.IN_PASSING) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, PASSING_COLOR);
            }
        } else if (ledTheme == LEDTheme.IN_SHOOTER) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, SHOOTER_COLOR);
            }
        } else if (ledTheme == LEDTheme.AUTO) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                if (i% 2 ==0){ledBuffer.setLED(i, Color.kYellow);} 
                    else {ledBuffer.setLED(i, Color.kBlack);}
            }
        led.setData(ledBuffer);
        } else if (ledTheme == LEDTheme.EXPLODE) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, Color.kRed);
            led.setData(ledBuffer);
            }
        }


    }

}
