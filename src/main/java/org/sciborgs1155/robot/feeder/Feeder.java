package org.sciborgs1155.robot.feeder;

import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Feeder extends SubsystemBase implements AutoCloseable, Logged {
  private final FeederIO feeder;

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
    setDefaultCommand(runOnce(() -> feeder.set(0)).andThen(Commands.idle()));
  }

  public Command forwards() {
    return run(() -> feeder.set(POWER));
  }

  public Command backwards() {
    return run(() -> feeder.set(-POWER / 2));
  }

  public Trigger noteAtShooter() {
    return new Trigger(feeder::beambreak);
  }

  public double getVelocity() {
    return feeder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    feeder.close();
  }
}
