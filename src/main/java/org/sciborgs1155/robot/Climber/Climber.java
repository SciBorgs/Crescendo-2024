package org.sciborgs1155.robot.Climber;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogFile;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Climber extends SubsystemBase implements Logged{
  //need to add measurements for clamping(don't want to overextend the climber)

  //sim objects
    //doubles - width, height
  Mechanism2d mech = new Mechanism2d();
  //name, length, angle
  MechanismLigament2d ligament = new MechanismLigament2d();
  //name
  MechanismObject2d object = new MechanismObject2d();
  //"anchor point"
  MechanismRoot2d root;

  // motor setup
  private final CANSparkMax motor = new CANSparkMax(sparkPort, MotorType.kBrushless);
  // private final CANSparkFlex 
  private final RelativeEncoder encoder = motor.getEncoder();
    

  // pid and ff controllers
  private final PIDController pid = new PIDController(kP, kI, kD);
  private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  //setVoltage or speed?? - DutyCyleEncoders?
  public void setMotorSpeed(double speed){
    motor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  // run when first scheduled
  public void climberInit() {
    // sets position to be 0 (remember to keep climber retracted at first)
    encoder.setPosition(0.0);
  }

  public Command stopExtending() {
    return run(() -> setMotorSpeed(0.0));
  }

  public Command fullyExtend() {
    moveToSetpoint(MAX_EXTENSION);
  }

  public Command retract(){
    return moveToSetpoint(0);
  }

  public Command moveToSetpoint(double setpoint){
    return run(() -> setMotorSpeed(ff.calculate(encoder.getVelocity()) + pid.calculate(encoder.getPosition(), setpoint))).withName("moving to setpoint");
  }
    
      
  @Override
  public void simulationPeriodic() {
    
      
  }
}
