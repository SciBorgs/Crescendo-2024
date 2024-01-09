package org.sciborgs1155.robot.Climber;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public Climber() {}

  double ffOutput;
  double fbOutput;

  // motor setup
  CANSparkMax motor = new CANSparkMax(sparkPort, MotorType.kBrushless);
  RelativeEncoder encoder;

  // pid and ff controllers
  PIDController pid = new PIDController(kP, kI, kD);
  ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  public void setDesiredSetpoint(double setpoint) {
    
  }

  public void setGoal(double goal) {
    
  }

  public void stopExtending() {
    motor.set(0.0);
  }

  public void retract() {}

  public void fullyExtend() {}

  // run when first scheduled
  public void climberInit() {
    // sets position to be 0 (remember to keep climber retracted at first)
    encoder.setPosition(0.0);

    ffOutput = ff.calculate(encoder.getVelocity());
    fbOutput = pid.calculate(encoder.getPosition(), goal)
  }

  
  public void getHeight(){

  }

  @Override
  public void periodic() {
    pid.calculate(encoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {}
}
