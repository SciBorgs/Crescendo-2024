package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.intake.IntakeConstants.DEBOUNCE_TIME;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.commands.NoteVisualizer;

public class Intake extends SubsystemBase implements Logged, AutoCloseable {
  public static Intake create() {
    return Robot.isReal() ? new Intake(new RealIntake()) : new Intake(new SimIntake());
  }

  public static Intake none() {
    return new Intake(new NoIntake());
  }

  private final IntakeIO hardware;

  private final EventLoop intakeTriggerPoller = new EventLoop();
  private final Trigger intakeTrigger;

  public Intake(IntakeIO hardware) {
    this.hardware = hardware;

    intakeTrigger = new Trigger(intakeTriggerPoller, hardware::seenNote);
    intakeTrigger.onFalse(stop());
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

  public Command runIntake(double power) {
    return runOnce(() -> hardware.setPower(power))
        .andThen(Commands.idle(this))
        .finallyDo(() -> hardware.setPower(0));
  }

  /**
   * Runs the intake forward.
   *
   * @return A command to run the intake.
   */
  public Command forward() {
    return runIntake(IntakeConstants.INTAKE_SPEED).withName("forward");
  }

  /**
   * Runs the intake backwards.
   *
   * @return A command to run the intake.
   */
  public Command backward() {
    return runIntake(-IntakeConstants.INTAKE_SPEED).withName("backward");
  }

  public Command stop() {
    return runIntake(0);
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
  public boolean stalling() {
    return hardware.current() > DCMotor.getNeoVortex(1).stallCurrentAmps;
  }

  public void pollTrigger() {
    intakeTriggerPoller.poll();
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
