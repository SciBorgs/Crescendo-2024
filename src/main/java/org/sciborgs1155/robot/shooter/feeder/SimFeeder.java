package org.sciborgs1155.robot.shooter.feeder;

import static org.sciborgs1155.robot.shooter.ShooterConstants.Feeder.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimFeeder implements FeederIO {
  DCMotor motor = DCMotor.getNEO(1);

  private final DCMotorSim simMotor =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA), motor, GEARING);

  // @Override public double getFeederVoltage(){
  //     motor.getVoltage()
  // }

  @Override
  public void setVoltage(double voltage) {
    simMotor.setInputVoltage(voltage);
  }
}
