package org.sciborgs1155.robot.shooter.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import monologue.Annotations.Log;
import org.sciborgs1155.robot.Constants;

public class SimPivot implements PivotIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              DCMotor.getNEO(2), SingleJointedArmSim.estimateMOI(LENGTH, MASS), GEARING),
          DCMotor.getNEO(2),
          GEARING,
          LENGTH,
          MIN_ANGLE.in(Radians),
          MAX_ANGLE.in(Radians),
          true,
          STARTING_ANGLE.in(Radians));

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(Constants.PERIOD.in(Seconds));
  }

  @Log.NT
  @Override
  public double getPosition() {
    return sim.getAngleRads();
  }

  @Override
  public void close() throws Exception {}
}
