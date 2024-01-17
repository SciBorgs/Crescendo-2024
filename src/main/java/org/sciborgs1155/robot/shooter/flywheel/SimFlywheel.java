package org.sciborgs1155.robot.shooter.flywheel;

import static org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel.GEARING;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel.MOI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel.*;

public class SimFlywheel implements FlywheelIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), MOI, GEARING),
          DCMotor.getNeoVortex(1),
          GEARING);

  private final PIDController pid = new PIDController(PID.kP, PID.kI, PID.kD);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FF.kS, FF.kV, FF.kA);

  private State lastSetpoint = new State();

  @Override
  public void close() throws Exception {}

  @Override
  public void setSetpoint(State setpoint) {
    double feedforward = ff.calculate(lastSetpoint.velocity, setpoint.velocity, Constants.PERIOD);
    double feedback = pid.calculate(getVelocity(), setpoint.velocity);

    sim.setInputVoltage(feedback + feedforward);
    lastSetpoint = setpoint;
  }

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return sim.getAngularVelocityRadPerSec();
  }
}
