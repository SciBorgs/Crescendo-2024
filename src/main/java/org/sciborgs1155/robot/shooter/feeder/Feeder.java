package org.sciborgs1155.robot.shooter.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Feeder extends SubsystemBase implements AutoCloseable, Logged {
  private final FeederIO feeder;

  public Feeder(FeederIO feeder) {
    this.feeder = feeder;
  }

  public static Feeder create() {
    return Robot.isReal() ? new Feeder(new RealFeeder()) : new Feeder(new SimFeeder());
  }

  public Command runFeeder(double voltage) {
    return run(() -> feeder.set(voltage)).withName("running feeder, " + voltage + " volts");
  }

  public Trigger frontBeambreakTrigger() {
    return new Trigger(feeder::getFrontBeambreakValue);
  }

  public Trigger endBeambreakTrigger() {
    return new Trigger(feeder::getEndBeambreakValue);
  }

  @Override
  public void close() throws Exception {
    feeder.close();
  }
}
