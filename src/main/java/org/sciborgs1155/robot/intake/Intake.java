package org.sciborgs1155.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import org.sciborgs1155.robot.intake.IntakeIO.RealIntake;
import org.sciborgs1155.robot.intake.IntakeIO.NoIntake;

public class Intake extends SubsystemBase implements Logged, AutoCloseable {
  private final IntakeIO intake;

  public Intake(IntakeIO intake) {
    this.intake = intake;
    setDefaultCommand(run(() -> intake.setPower(0)));
  }

  public Command intake() {
    return run(() -> intake.setPower(IntakeConstants.INTAKE_SPEED));
  }

  public Command outtake() {
    return run(() -> intake.setPower(-IntakeConstants.INTAKE_SPEED));
  }

  public Trigger holdingNote() {
    return new Trigger(intake::getBeambreakStatus);
  }

  public static Intake create(boolean isReal) {
    return isReal ? new Intake(new RealIntake()) : new Intake(new NoIntake());
  }

  @Override
  public void close() throws Exception {
      intake.close();
  }
}
