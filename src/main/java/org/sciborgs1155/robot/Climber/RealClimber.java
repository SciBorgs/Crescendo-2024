package org.sciborgs1155.robot.climber;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;
import static org.sciborgs1155.robot.climber.ClimberConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class RealClimber implements ClimberIO {

  CANSparkMax motor = new CANSparkMax(sparkPort, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setPosition(double position) {
    encoder.setPosition(position);
  }
}
