package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.shooter.ShooterConstants.GEARING;
import static org.sciborgs1155.robot.shooter.ShooterConstants.kA;
import static org.sciborgs1155.robot.shooter.ShooterConstants.kV;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.sciborgs1155.robot.Constants;

/** A simulated {@link ShooterIO} using {@link FlywheelSim} */
public class SimShooter implements ShooterIO {
  private final FlywheelSim top =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(kV, kA), DCMotor.getNeoVortex(2), GEARING);

  private final FlywheelSim bottom =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(kV, kA), DCMotor.getNeoVortex(2), GEARING);

  @Override
  public void close() throws Exception {}

  @Override
  public void setSetpoint(double velocity) {
    top.setState(velocity);
    bottom.setState(velocity);
  }

  @Override
  public void setVoltage(double voltage) {
    top.setInputVoltage(voltage);
    top.update(Constants.PERIOD.in(Seconds));

    bottom.setInputVoltage(voltage);
    bottom.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double topVelocity() {
    return top.getAngularVelocityRadPerSec();
  }

  @Override
  public double bottomVelocity() {
    return bottom.getAngularVelocityRadPerSec();
  }

  @Override
  public boolean atSetpoint() {
    return true;
  }

  @Override
  public double setpoint() {
    return (topVelocity() + bottomVelocity()) / 2.0;
  }
}
