package org.sciborgs1155.robot.shooter.flywheel;

import static org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel.GEARING;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel.MOI;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SimFlywheel implements FlywheelIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), MOI, GEARING),
          DCMotor.getNeoVortex(1),
          GEARING);

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return sim.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
