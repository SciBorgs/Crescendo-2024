package org.sciborgs1155.robot.Climber;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;
import static org.sciborgs1155.robot.Climber.ClimberConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;

public class SimClimber extends SubsystemBase implements Logged, ClimberIO {
  //most of this is copied from climber.java atp, PENDING CHANGES ARE BEING MADE

  // need to add measurements for clamping(don't want to overextend the climber)

  // motor setup
  private final CANSparkMax motor = new CANSparkMax(sparkPort, MotorType.kBrushless);
  // private final CANSparkFlex
  private final RelativeEncoder encoder = motor.getEncoder();

  // pid and ff controllers
  @LogBoth
  private final ProfiledPIDController Simpid =
      new ProfiledPIDController(
          kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

  @LogBoth private final ElevatorFeedforward Simff = new ElevatorFeedforward(kS, kG, kV, kA);

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
                Simff.calculate(encoder.getVelocity()) + Simpid.calculate(encoder.getPosition(), goal)))
        .withName("moving to setpoint");
  }


// sim objects
    // doubles - width, height
    private Mechanism2d mech = new Mechanism2d(50, 50);
    // name, length, angle
    // private MechanismLigament2d ligament = new MechanismLigament2d();
    // // name
    // private MechanismObject2d climber = new MechanismObject2d("climber");
    // "anchor point"
    private MechanismRoot2d climberBase = mech.getRoot("climberBase", 25.0, 0.0);

  @Override
  public void simulationPeriodic(){
    MechanismObject2d ligament = climberBase.append(new MechanismLigament2d("climber", MININMUM_CLIMBER_LENGTH, 90));
  }
}
