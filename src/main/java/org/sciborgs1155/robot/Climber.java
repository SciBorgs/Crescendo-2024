package org.sciborgs1155.robot;

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

    CANSparkMax motor = new CANSparkMax(sparkPort, MotorType.kBrushless);
    RelativeEncoder encoder;
    

    PIDController pid = new PIDController(kP, kI, kD);
    ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

    public void setDesiredSpeed(double speed){
        motor.set(speed);
    }

    public void setGoal(double goal){
        setDesiredSpeed(pid.calculate(goal) + ff.calculate(goal));
    }

    @Override
    public void periodic(){
        pid.calculate(encoder.getPosition());
    }

    @Override
    public void simulationPeriodic(){

    }


}