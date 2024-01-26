package org.sciborgs1155.robot.shooter.feeder;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.shooter.ShooterConstants.FeederConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.sciborgs1155.robot.Constants;

public class SimFeeder implements FeederIO {
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA), DCMotor.getNeoVortex(1), GEARING);

  @Override
  public void set(Measure<Velocity<Distance>> speed) {
    //fix speed to voltage conversion later
    sim.setInputVoltage(speed.in(Meters.per(Second)));
    sim.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public void close() throws Exception {}

  @Override
  public Measure<Velocity<Distance>> getVelocity() {
    return MetersPerSecond.of(sim.getAngularVelocityRadPerSec());
  }
}
