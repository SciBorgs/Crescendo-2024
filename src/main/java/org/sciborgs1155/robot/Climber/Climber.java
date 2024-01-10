package org.sciborgs1155.robot.Climber;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;

public class Climber extends SubsystemBase implements Logged, ClimberIO {
  // need to add measurements for clamping(don't want to overextend the climber)

  // sim objects
  // doubles - width, height
  // Mechanism2d mech = new Mechanism2d();
  // // name, length, angle
  // MechanismLigament2d ligament = new MechanismLigament2d();
  // // name
  // MechanismObject2d object = new MechanismObject2d();
  // // "anchor point"
  // MechanismRoot2d root;

  // motor setup
  private final CANSparkMax motor = new CANSparkMax(sparkPort, MotorType.kBrushless);
  // private final CANSparkFlex
  private final RelativeEncoder encoder = motor.getEncoder();

  // pid and ff controllers
  // setup Profiled PID later
  @LogBoth
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

  @LogBoth private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  // setVoltage or speed?? - DutyCyleEncoders?
  public void setMotorSpeed(double speed) {
    motor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  // run when first scheduled
  @Override
  public void climberInit() {
    // sets position to be 0 (remember to keep climber retracted at first)
    encoder.setPosition(0.0);
  }

  @Override
  public Command stopExtending() {
    return run(() -> setMotorSpeed(0.0));
  }

  @Override
  public Command fullyExtend() {
    return moveToGoal(MAX_EXTENSION);
  }

  @Override
  public Command retract() {
    return moveToGoal(0);
  }

  @Override
  public Command moveToGoal(double goal) {
    return run(() ->
            setMotorSpeed(
                ff.calculate(encoder.getVelocity()) + pid.calculate(encoder.getPosition(), goal)))
        .withName("moving to setpoint");
  }
}
