package org.sciborgs1155.robot.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;

public class Climber extends SubsystemBase{
    public Climber(){
        
    }
    //motor setup
    CANSparkMax motor = new CANSparkMax(sparkPort, MotorType.kBrushless);
    RelativeEncoder encoder;

    //pid and ff controllers
    PIDController pid = new PIDController(kP, kI, kD);
    ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

    public void setDesiredSpeed(double speed){
        motor.set(speed);
    }

    public void setGoal(double goal){
        setDesiredSpeed(pid.calculate(encoder.getPosition(), goal) + ff.calculate(goal));

    }

    public void stopExtending(){
        motor.set(0.0);
    }

    public void retract(){

    }

    public void fullyExtend(){

    }

    //run when first scheduled
    public void climberInit(){
        //sets position to be 0 (remember to keep climber retracted at first)
        encoder.setPosition(0.0);
    }

    @Override
    public void periodic(){
        pid.calculate(encoder.getPosition());
    }

    @Override
    public void simulationPeriodic(){

    }


}