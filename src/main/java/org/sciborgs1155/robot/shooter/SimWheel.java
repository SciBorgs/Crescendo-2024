package org.sciborgs1155.robot.shooter;

import static org.sciborgs1155.robot.shooter.ShooterConstants.GEARING;
import static org.sciborgs1155.robot.shooter.ShooterConstants.kA;
import static org.sciborgs1155.robot.shooter.ShooterConstants.kV;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SimWheel implements WheelIO {
  private final FlywheelSim flywheel =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(kV, kA), DCMotor.getNeoVortex(2), GEARING);

  @Override
  public void setVoltage(double voltage) {
    flywheel.setInputVoltage(voltage);
  }

  @Override
  public double velocity() {
    return flywheel.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}

  @Override
  public void setInverted(boolean inverted) {}
}
