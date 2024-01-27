package org.sciborgs1155.robot.shooter.feeder;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public Command runFeeder(Measure<Voltage> voltage) {
    return run(() -> feeder.set(voltage)).withName("running Feeder");
  }

  //   public void runFeederBase(Measure<Voltage> voltage) {
  //     feeder.set(voltage);
  //   }

  public Command runFeederInverse(Measure<Voltage> voltage) {
    voltage = Volts.of(voltage.in(Volts) * -1);
    return runFeeder(voltage).withName("running Feeder backwards");
  }

  //   public void runInverseFeederBase(Measure<Voltage> voltage) {
  //     voltage = Volts.of(voltage.in(Volts) * -1);
  //     runFeederBase(voltage);

  //   }

  public Measure<Voltage> getVoltage() {
    return Volts.of(feeder.getVoltage().in(Volts));
  }

  @Override
  public void close() throws Exception {
    feeder.close();
  }
}
