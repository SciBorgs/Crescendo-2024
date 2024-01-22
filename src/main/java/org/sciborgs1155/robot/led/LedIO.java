package org.sciborgs1155.robot.led;

import org.sciborgs1155.robot.led.Led.LEDTheme;

import monologue.Logged;

public interface LedIO extends Logged {
    public void setTheme(LEDTheme ledtheme);

}
