package org.sciborgs1155.robot.shooter.flywheel;

import static org.sciborgs1155.robot.Ports.Shooter.Flywheel.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import org.sciborgs1155.robot.Constants;

public class RealFlywheel implements FlywheelIO {
  private final CANSparkFlex flywheel = new CANSparkFlex(FLYWHEEL, MotorType.kBrushless);
  private final RelativeEncoder encoder = flywheel.getEncoder();

  private final PIDController pid = new PIDController(PID.kP, PID.kI, PID.kD);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FF.kS, FF.kV, FF.kA);

  private State lastSetpoint = new State();

  public RealFlywheel() {
    flywheel.restoreFactoryDefaults();
    flywheel.setInverted(false);
    flywheel.setIdleMode(IdleMode.kBrake);
    flywheel.setSmartCurrentLimit(CURRENT_LIMIT);

    encoder.setPositionConversionFactor(-1);
    encoder.setVelocityConversionFactor(-1);

    flywheel.burnFlash();
  }

  @Override
  public void setSetpoint(State setpoint) {
    double feedforward = ff.calculate(lastSetpoint.velocity, setpoint.velocity, Constants.PERIOD);
    double feedback = pid.calculate(getVelocity(), setpoint.velocity);

    flywheel.setVoltage(feedback + feedforward);
    lastSetpoint = setpoint;
  }

  @Override
  public void setVoltage(double voltage) {
    flywheel.setVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    flywheel.close();
  }
}
