package org.sciborgs1155.robot.shooter.flywheel;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.shooter.ShooterConstants.FlywheelConstants.GEARING;
import static org.sciborgs1155.robot.shooter.ShooterConstants.FlywheelConstants.MOI;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.sciborgs1155.robot.Constants;

/** A simulated {@link FlywheelIO} using {@link FlywheelSim} */
public class SimFlywheel implements FlywheelIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), MOI, GEARING),
          DCMotor.getNeoVortex(2),
          GEARING);

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double getVelocity() {
    return sim.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
