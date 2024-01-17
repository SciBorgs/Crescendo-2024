package org.sciborgs1155.robot.shooter.feeder;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class RealFeeder implements FeederIO{
    CANSparkFlex motor  = new CANSparkFlex(0, MotorType.kBrushless)

    @Override
    public double getFeederSpeed(){

    }

    @Override
    public void setFeederVoltage(double speed){

    }
}
