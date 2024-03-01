package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.commands.NoteVisualizer;

public class Feeder extends SubsystemBase implements AutoCloseable, Logged {
  @Log.NT private final FeederIO feeder;

  /** Creates a real or non-existent feeder based on {@link Robot#isReal()}. */
  public static Feeder create() {
    return Robot.isReal() ? new Feeder(new RealFeeder()) : new Feeder(new NoFeeder());
  }

  /** Creates a non-existent feeder. */
  public static Feeder none() {
    return new Feeder(new NoFeeder());
  }

  public Feeder(FeederIO feeder) {
    this.feeder = feeder;
    setDefaultCommand(runOnce(() -> feeder.setPower(0)).andThen(Commands.idle()).withName("idle"));
  }

  public Command runFeeder(double power) {
    return runOnce(() -> feeder.setPower(power));
  }

  public Command forward() {
    return runFeeder(POWER).withName("forward");
  }

  public Command backward() {
    return runFeeder(-POWER).withName("backward");
  }

  /**
   * Runs the feeder forward until a note exits it.
   *
   * @return A command to eject a note from the feeder.
   */
  public Command eject() {
    return Commands.waitUntil(noteAtShooter())
        .andThen(Commands.waitUntil(noteAtShooter().negate()))
        .deadlineWith(forward())
        .alongWith(NoteVisualizer.shoot())
        .withName("eject");
  }

  public Command retract() {
    return backward().withTimeout(TIMEOUT.in(Seconds));
  }

  /**
   * Whether there currently is a note at the top of the feeder, touching the shooter.
   *
   * @return A trigger based on the upper feeder beambreak.
   */
  public Trigger noteAtShooter() {
    return new Trigger(feeder::beambreak)
        .negate()
        .debounce(DEBOUNCE_TIME.in(Seconds), DebounceType.kFalling);
  }

  @Override
  public void periodic() {
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    feeder.close();
  }
}
