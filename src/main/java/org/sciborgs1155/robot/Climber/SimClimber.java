package org.sciborgs1155.robot.climber;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;
import static org.sciborgs1155.robot.climber.ClimberConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        MININMUM_CLIMBER_LENGTH, 
        MAXIMUM_CLIMBER_LENGTH, 
true, 
0.0);

  // initialize encoder later / soon
  private final Encoder encoder = new Encoder(quadPortA, quadPortB);
  private final EncoderSim simEncoder = new EncoderSim(encoder);

  // run when first scheduled
  // quadrature encoder??
  // private final Encoder encoder1;

  // name, length, angle
  // private MechanismLigament2d ligament = new MechanismLigament2d();
  // // name
  // private MechanismObject2d climber = new MechanismObject2d("climber");
  // "anchor point"

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
  }

  @Override
  public void setPosition(double position) {}
}
