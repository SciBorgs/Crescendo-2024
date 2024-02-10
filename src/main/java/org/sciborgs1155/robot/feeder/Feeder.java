package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Feeder extends SubsystemBase implements AutoCloseable, Logged {
  private final FeederIO feeder;

  public Feeder(FeederIO feeder) {
    this.feeder = feeder;
  }

  /** Creates a real or non-existent feeder based on {@link Robot#isReal()}. */
  public static Feeder create() {
    return Robot.isReal() ? new Feeder(new RealFeeder()) : new Feeder(new NoFeeder());
  }

  /** Creates a non-existent feeder. */
  public static Feeder none() {
    return new Feeder(new NoFeeder());
  }

  public Command runFeeder(double power) {
    return run(() -> feeder.set(power));
  }

  public Measure<Velocity<Angle>> getVelocity() {
    return RadiansPerSecond.of(feeder.getVelocity());
  }

  @Override
  public void close() throws Exception {
    feeder.close();
  }
}
