package org.sciborgs1155.robot.shooter.feeder;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.shooter.ShooterConstants.FeederConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.sciborgs1155.robot.Constants;

public class SimFeeder implements FeederIO {
  Measure<Voltage> voltage = Volts.of(0);

  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA), DCMotor.getNeoVortex(1), GEARING);

  @Override
  public void set(Measure<Voltage> voltage) {
    // fix speed to voltage conversion later
    sim.setInputVoltage(voltage.in(Volts));
    this.voltage = voltage;
    sim.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public void close() throws Exception {}

  @Override
  public Measure<Voltage> getVoltage() {
    return voltage;
  }
}
