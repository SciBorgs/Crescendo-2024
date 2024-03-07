package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.sciborgs1155.robot.Constants;

/** A simulated {@link ShooterIO} using {@link FlywheelSim} */
public class SimShooter implements ShooterIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(kV, kA), DCMotor.getNeoVortex(2), GEARING);

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double velocity() {
    return sim.getAngularVelocityRadPerSec();
  }

  @Override
  public double current() {
    return sim.getCurrentDrawAmps();
  }

  @Override
  public void close() throws Exception {}
}
