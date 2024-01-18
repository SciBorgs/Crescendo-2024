// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.shooter.pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import org.sciborgs1155.robot.shooter.pivot.PivotIO;
import org.sciborgs1155.robot.Ports.Shooter.Pivot;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;

/** Add your docs here. */
public class RealPivot implements PivotIO{
    private final CANSparkFlex pivotMotor;
    public final Encoder pivotMotorEncoder;

    public RealPivot(){
        this.pivotMotor = new CANSparkFlex(Pivot.PIVOT_SPARK_ONE,MotorType.kBrushless);
        this.pivotMotorEncoder = new Encoder(0,Pivot.PIVOT_THROUGHBORE);

    }
    @Override
    public double getVoltage(){
        return pivotMotor.getBusVoltage();
    }
    @Override 
    public void setVoltage(double voltage){
        pivotMotor.setVoltage(voltage);
    }
}
