package org.sciborgs1155.robot.climber;

import static org.sciborgs1155.robot.climber.ClimberConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.sciborgs1155.robot.Constants;

public class SimClimber implements ClimberIO {
  // most of this is copied from climber.java atp, PENDING CHANGES ARE BEING MADE

  // need to add measurements for clamping(don't want to overextend the climber)

  // sim objects
  DCMotor motor = DCMotor.getNEO(1);
  // remember to put elevator sim,
  private ElevatorSim elevatorSim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(motor, CLIMBER_MASS, RADIUS_METERS, G),
          motor,
          MINIMUM_CLIMBER_LENGTH,
          MAXIMUM_CLIMBER_LENGTH,
          true,
          0.0);

  @Override
  public double getVelocity() {
    return elevatorSim.getVelocityMetersPerSecond();
  }

  @Override
  public double getPosition() {
    return elevatorSim.getPositionMeters();
  }

  @Override
  public void setVoltage(double voltage) {
    elevatorSim.setInputVoltage(voltage);
    elevatorSim.update(Constants.PERIOD);
  }

  @Override
  public void setPosition(double position) {}
}
