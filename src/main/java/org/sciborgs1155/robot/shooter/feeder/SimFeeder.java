package org.sciborgs1155.robot.shooter.feeder;

import static org.sciborgs1155.robot.shooter.ShooterConstants.FeederConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimFeeder implements FeederIO {
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA), DCMotor.getNeoVortex(1), GEARING);

  @Override
  public void set(double speed) {
    sim.setInputVoltage(speed);
  }

  @Override
  public void close() throws Exception {}

  @Override
  public double getVelocity() {
    return sim.getAngularVelocityRadPerSec();
  }
}
