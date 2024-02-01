package org.sciborgs1155.robot.shooter.feeder;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.shooter.ShooterConstants.FeederConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
  public void close() throws Exception {}

  @Override
  public boolean getFrontBeambreakValue() {
    return false;
  }

  @Override
  public boolean getEndBeambreakValue() {
    return false;
  }
}
