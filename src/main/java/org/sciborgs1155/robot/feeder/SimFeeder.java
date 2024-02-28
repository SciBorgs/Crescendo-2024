package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import monologue.Annotations.Log;
import org.sciborgs1155.robot.Constants;

public class SimFeeder implements FeederIO {
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA), DCMotor.getNeoVortex(1), GEARING);

  @Override
  public void set(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  @Log.NT
  public boolean beambreak() {
    return true;
  }

  @Override
  public double current() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
