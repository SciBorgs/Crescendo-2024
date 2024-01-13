package org.sciborgs1155.robot.climber;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;
import static org.sciborgs1155.robot.climber.ClimberConstants.*;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;

public class Climber extends SubsystemBase implements Logged {

  public Climber() {  }

  // need to add measurements for clamping(don't want to overextend the climber)

  // sim objects

  private Mechanism2d mech;
  private MechanismRoot2d root;

  private MechanismLigament2d climberTrunk;
  private MechanismLigament2d climberHook;
  private MechanismRoot2d climberBase = mech.getRoot("climberBase", 25.0, 0.0);

  private SmartDashboard dashboard;


  // hardware IOs
  ClimberIO climber = Robot.isReal() ? new RealClimber() : new SimClimber();

  // motor setup
  // private final CANSparkMax motor = new CANSparkMax(sparkPort, MotorType.kBrushless);
  // // private final CANSparkFlex
  // private final RelativeEncoder encoder = motor.getEncoder();

  // pid and ff controllers
  @LogBoth
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

  @LogBoth private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

   // run when first scheduled
  public void climberInit() {
    // sets position to be 0 (remember to keep climber retracted at first)
    climber.setPosition(0.0);

    mech = new Mechanism2d(50, 50);
    root = mech.getRoot("climber", 25.0, 0.0);

    climberTrunk = root.append(new MechanismLigament2d("climbTrunk", MININMUM_CLIMBER_LENGTH, 90));
    climberHook = climberTrunk.append(new MechanismLigament2d("hook", HOOK_LENGTH, 90));

    SmartDashboard.putData("mech2d", mech);
  }

  // setVoltage or speed?? - DutyCyleEncoders?
  public void setMotorVoltage(double voltage) {
    climber.setVoltage(voltage);
  }

  public Command stopExtending() {
    return run(() -> setMotorVoltage(0.0));
  }

  public Command fullyExtend() {
    return moveToGoal(MAX_EXTENSION);
  }

  public Command retract() {
    return moveToGoal(0);
  }

  public Command moveToGoal(double goal) {
    pid.setGoal(goal);

    return run(() ->
            setMotorVoltage(
                ff.calculate(climber.getVelocity())
                    + pid.calculate(climber.getPosition(), goal)))
        .withName("moving to Goal");
  }

  @Override
  public void simulationPeriodic(){
    climberTrunk.setLength(climber.getPosition());
  }
}
