package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.shooter.ShooterConstants.GEARING;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.sciborgs1155.robot.shooter.ShooterConstants.Top;

public class SimWheel implements WheelIO {
  private final FlywheelSim flywheel =
      new FlywheelSim( // TODO fix
          LinearSystemId.identifyVelocitySystem(Top.kV, Top.kA), DCMotor.getNeoVortex(2), GEARING);

  @Override
  public void setVoltage(double voltage) {
    flywheel.setInputVoltage(voltage);
    flywheel.update(PERIOD.in(Seconds));
  }

  @Override
  public double velocity() {
    return flywheel.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
