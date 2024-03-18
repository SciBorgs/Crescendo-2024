package org.sciborgs1155.robot.shooter;

import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** A simulated {@link ShooterIO} using {@link FlywheelSim} */
public class SimShooter implements ShooterIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(kV, kA), DCMotor.getNeoVortex(2), GEARING);

  // TODO

  @Override
  public void close() throws Exception {}

  @Override
  public void setSetpoint(double velocity) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double topVelocity() {
    return 0;
  }

  @Override
  public double bottomVelocity() {
    return 0;
  }

  @Override
  public boolean atSetpoint() {
    return false;
  }
}
