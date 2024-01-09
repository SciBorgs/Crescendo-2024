package org.sciborgs1155.robot.Climber;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  //need to add measurements for clamping(don't want to overextend the climber)

  // motor setup
  CANSparkMax motor = new CANSparkMax(sparkPort, MotorType.kBrushless);
  RelativeEncoder encoder;

  // pid and ff controllers
  PIDController pid = new PIDController(kP, kI, kD);
  ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  public void setMotorSpeed(double speed){
    motor.set(speed);
  }

//   public void stopExtending() {
    
//   }

  public void retract() {
    moveToSetpoint(0);
  }

//   public void fullyExtend() {
//     moveToSetpoint();
//   }

  // run when first scheduled
  public void climberInit() {
    // sets position to be 0 (remember to keep climber retracted at first)
    encoder.setPosition(0.0);
  }

  public Command moveToSetpoint(double setpoint){
    return run(() -> setMotorSpeed(ff.calculate(encoder.getVelocity()) + pid.calculate(encoder.getPosition(), setpoint)));
  }
    

//   @Override
//   public void periodic() {
    
//   }

  @Override
  public void simulationPeriodic() {}
}
