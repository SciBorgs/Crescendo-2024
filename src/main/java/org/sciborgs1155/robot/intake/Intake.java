package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.intake.IntakeConstants.DEBOUNCE_TIME;
import static org.sciborgs1155.robot.intake.IntakeConstants.STALL_THRESHOLD;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import monologue.Logged;
import monologue.Annotations.Log;

import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.commands.NoteVisualizer;

public class Intake extends SubsystemBase implements Logged, AutoCloseable {
  public static Intake create() {
    return Robot.isReal() ? new Intake(new RealIntake()) : new Intake(new NoIntake());
  }

  public static Intake none() {
    return new Intake(new NoIntake());
  }

  private final IntakeIO hardware;

  public Intake(IntakeIO hardware) {
    this.hardware = hardware;
    setDefaultCommand(
        runOnce(() -> hardware.setPower(0)).andThen(Commands.idle()).withName("idle"));
  }

  /**
   * Runs the intake forward until a note enters and exits the intake.
   *
   * @return A command to run the intake.
   */
  public Command intake() {
    return Commands.waitUntil(hasNote())
        .andThen(Commands.waitUntil(hasNote().negate()))
        .deadlineWith(forward())
        .alongWith(NoteVisualizer.intake())
        .withName("intaking");
  }

  /**
   * Runs the intake forward.
   *
   * @return A command to run the intake.
   */
  public Command forward() {
    return run(() -> hardware.setPower(IntakeConstants.INTAKE_SPEED)).withName("forward");
  }

  /**
   * Runs the intake backwards.
   *
   * @return A command to run the intake.
   */
  public Command backward() {
    return run(() -> hardware.setPower(-IntakeConstants.INTAKE_SPEED)).withName("backward");
  }



  /**
   * Whether the intake currently contains a note.
   *
   * @return A trigger based on the intake beambreak.
   */
  public Trigger hasNote() {
    return new Trigger(hardware::beambreak)
        .negate()
        .debounce(DEBOUNCE_TIME.in(Seconds), DebounceType.kFalling);
  }

  @Log.NT
  public boolean stallingCurrent() {
    return hardware.current() > STALL_THRESHOLD.in(Amp);
  }

  @Override
  public void periodic() {
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
