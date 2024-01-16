package org.sciborgs1155.robot.climber;

import static org.sciborgs1155.robot.Ports.ClimberPorts.*;
import static org.sciborgs1155.robot.climber.ClimberConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import org.sciborgs1155.robot.Robot;

public class Climber extends SubsystemBase implements Logged {

  public Climber() {
    // sets position to be 0 (remember to keep climber retracted at first)
    climber.setPosition(0.0);

    SmartDashboard.putData("mech2d", mech);
  }

  // sim objects
  @LogBoth private Mechanism2d mech = new Mechanism2d(6, 9, new Color8Bit(Color.kAntiqueWhite));
  @LogBoth private MechanismRoot2d root = mech.getRoot("climber", 3, 2);
  @LogBoth private MechanismRoot2d climberBase = mech.getRoot("climberBase", 25.0, 0.0);

  @LogBoth
  double position() {
    return climber.getPosition();
  }

  private MechanismLigament2d climberTrunk =
      root.append(
          new MechanismLigament2d(
              "climbTrunk", MINIMUM_CLIMBER_LENGTH, 90, 6, new Color8Bit(Color.kBlueViolet)));
  private MechanismLigament2d climberHook =
      climberTrunk.append(
          new MechanismLigament2d("hook", HOOK_LENGTH, 90, 6, new Color8Bit(Color.kFirebrick)));

  // hardware IOs
  ClimberIO climber = Robot.isReal() ? new RealClimber() : new SimClimber();

  // pid and ff controllers
  @LogBoth
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

  @LogBoth private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  public Command stopExtending() {
    return run(() -> climber.setVoltage(0.0));
  }

  public Command fullyExtend() {
    return moveToGoal(MAXIMUM_CLIMBER_LENGTH);
  }

  public Command retract() {
    return moveToGoal(MINIMUM_CLIMBER_LENGTH);
  }

  public Command moveToGoal(double goal) {
    return run(() ->
            climber.setVoltage(
                pid.calculate(climber.getPosition(), goal)
                    + ff.calculate(pid.getSetpoint().velocity)))
        .withName("moving to Goal");
  }

  @Override
  public void simulationPeriodic() {
    climberTrunk.setLength(climber.getPosition());
  }
}
