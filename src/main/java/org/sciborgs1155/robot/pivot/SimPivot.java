package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.robot.Constants;

public class SimPivot implements PivotIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              DCMotor.getNEO(4),
              MOI.in((Meters).mult(Meters).mult(Kilograms)),
              1.0 / MOTOR_GEARING),
          DCMotor.getNEO(4),
          1.0 / MOTOR_GEARING,
          -LENGTH.in(Meters),
          MIN_ANGLE.getRadians(),
          MAX_ANGLE.getRadians(),
          true,
          STARTING_ANGLE.getRadians());

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double getPosition() {
    return sim.getAngleRads();
  }

  @Override
  public double getVelocity() {
    return sim.getVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
