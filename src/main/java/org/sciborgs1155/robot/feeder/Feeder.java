package org.sciborgs1155.robot.feeder;

import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Feeder extends SubsystemBase implements AutoCloseable, Logged {
  private final FeederIO feeder;

  private final PIDController pid = new PIDController(kP, kI, kD);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

  public Feeder(FeederIO feeder) {
    this.feeder = feeder;
  }

  public static Feeder create() {
    return Robot.isReal() ? new Feeder(new RealFeeder()) : new Feeder(new SimFeeder());
  }

  public Command runFeeder(double voltage) {
    return run(() ->
            feeder.set(pid.calculate(feeder.getVelocity(), voltage) + ff.calculate(voltage)))
        .withName("running feeder, " + voltage + " volts");
  }

  @Override
  public void close() throws Exception {
    feeder.close();
  }
}
