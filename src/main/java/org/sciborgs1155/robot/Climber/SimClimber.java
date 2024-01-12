package org.sciborgs1155.robot.climber;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;
import static org.sciborgs1155.robot.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimClimber implements ClimberIO {
  // most of this is copied from climber.java atp, PENDING CHANGES ARE BEING MADE

  // need to add measurements for clamping(don't want to overextend the climber)

  // sim objects

  // remember to put elevator sim

  private MechanismLigament2d climberTrunk;
  private MechanismLigament2d climberHook;

  private Mechanism2d mech;
  private MechanismRoot2d root;

  private SmartDashboard dashboard;

  // initialize encoder later / soon
  private final Encoder encoder = new Encoder(quadPortA, quadPortB);
  private final EncoderSim simEncoder = new EncoderSim(encoder);

  // run when first scheduled
  @Override
  public void initialize() {
    // sets position to be 0 (remember to keep climber retracted at first)
    encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    // doubles - width, height
    mech = new Mechanism2d(50, 50);
    root = mech.getRoot("climber", 25.0, 0.0);

    climberTrunk = root.append(new MechanismLigament2d("climbTrunk", MININMUM_CLIMBER_LENGTH, 90));
    climberHook = climberTrunk.append(new MechanismLigament2d("hook", HOOK_LENGTH, 90));

    dashboard.putData("mech2d", mech);
  }

  // quadrature encoder??
  // private final Encoder encoder1;

  // name, length, angle
  // private MechanismLigament2d ligament = new MechanismLigament2d();
  // // name
  // private MechanismObject2d climber = new MechanismObject2d("climber");
  // "anchor point"
  private MechanismRoot2d climberBase = mech.getRoot("climberBase", 25.0, 0.0);

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void set(double speed) {
    motor.set(speed);
  }

  @Override
  public void setPosition(double position) {
    encoder.setPosition(position);
  }

  @Override
  public void simulationPeriodic() {}
}
