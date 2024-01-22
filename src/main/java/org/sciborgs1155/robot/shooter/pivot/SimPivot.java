package org.sciborgs1155.robot.shooter.pivot;

import static org.sciborgs1155.robot.shooter.ShooterConstants.Pivot.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimPivot implements PivotIO {
  private static final Unit<Angle> Radians = null;
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
  }

  @Override
  public double getPosition() {
    return sim.getAngleRads();
  }

  @Override
  public void close() throws Exception {}
}
