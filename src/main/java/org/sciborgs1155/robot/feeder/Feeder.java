package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  }

  public Command runFeeder(double power) {
    return runOnce(() -> feeder.set(power)).andThen(Commands.idle()).finallyDo(() -> feeder.set(0));
  }

  public Command forward() {
    return runFeeder(POWER).alongWith(NoteVisualizer.shoot());
  }

  public Command backward() {
    return runFeeder(-POWER / 2.0);
  }

  public Command retract() {
    return backward().withTimeout(0.15);
  }

  public Trigger atShooter() {
    return new Trigger(() -> !feeder.beambreak())
        .debounce(PERIOD.times(3).in(Seconds), DebounceType.kBoth);
  }

  @Log.NT
  public double getVelocity() {
    return feeder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    feeder.close();
  }
}
