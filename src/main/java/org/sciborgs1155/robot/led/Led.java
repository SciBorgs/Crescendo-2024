package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Constants.Led.INTAKE_COLOR;
import static org.sciborgs1155.robot.Constants.Led.LEDLENGTH;
import static org.sciborgs1155.robot.Constants.Led.PASSING_COLOR;
import static org.sciborgs1155.robot.Constants.Led.SHOOTER_COLOR;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;

public class Led extends SubsystemBase implements Logged, AutoCloseable{
    private final LedIO led = Robot.isReal() ? new RealLed() : new SimLed(); //led as a class
    public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLENGTH); 

    public static enum LEDTheme{
        RAINBOW,
        BXSCI,
        IN_INTAKE,
        IN_PASSING,
        IN_SHOOTER,
        AUTO,
        EXPLODE //means error now, unless you want to implement the ability for the robot to explode (potentially high risk high reward strategy)
    }

    static double time = 0;
    private LEDTheme ledThemeNow = LEDTheme.AUTO; 
    public void setLEDTheme(LEDTheme ledTheme){
        ledThemeNow=ledTheme;
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
        } else if (ledTheme == LEDTheme.EXPLODE) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, Color.kRed);
            }
        }

        led.setData(ledBuffer);

        //documentation: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
        //some inspiration by: https://github.com/SciBorgs/ChargedUp-2023/blob/io-rewrite/src/main/java/org/sciborgs1155/robot/subsystems/LED.java  
    }
    //work on commands and stuff

    @Log.NT
    public LEDTheme getTheme(){
        return ledThemeNow;
    }

    public Command setTheme(LEDTheme ledTheme){
        return run(()->setLEDTheme(ledTheme));
    }

    @Override
    public void close() throws Exception {
        led.close();
    }
}
