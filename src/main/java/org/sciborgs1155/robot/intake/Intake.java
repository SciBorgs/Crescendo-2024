package org.sciborgs1155.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.intake.IntakeIO.NoIntake;
import org.sciborgs1155.robot.intake.IntakeIO.RealIntake;

public class Intake extends SubsystemBase implements Logged, AutoCloseable {
  public static Intake create() {
    return Robot.isReal() ? new Intake(new RealIntake()) : new Intake(new NoIntake());
  }

  public static Intake none() {
    return new Intake(new NoIntake());
  }

  private final IntakeIO intake;

  public Intake(IntakeIO intake) {
    this.intake = intake;
    setDefaultCommand(runOnce(() -> intake.setPower(0)).andThen(Commands.idle()));
  }

  public Command intake() {
    return run(() -> intake.setPower(IntakeConstants.INTAKE_SPEED));
  }

  public Command outtake() {
    return run(() -> intake.setPower(-IntakeConstants.INTAKE_SPEED));
  }

  // public Trigger hasNote() {
  //   return new Trigger(intake::beamBreak);
  // }

  @Override
  public void close() throws Exception {
    intake.close();
  }
}
