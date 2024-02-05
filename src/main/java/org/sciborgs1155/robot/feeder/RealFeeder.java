package org.sciborgs1155.robot.feeder;

import static org.sciborgs1155.robot.Ports.Feeder.*;
import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealFeeder implements FeederIO {
  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;

  public RealFeeder() {
    motor = SparkUtils.createSparkFlex(FEEDER_SPARK, false, IdleMode.kBrake, CURRENT_LIMIT);

    SparkUtils.configureFrameStrategy(
        motor, Set.of(Data.POSITION, Data.VELOCITY, Data.OUTPUT), Set.of(Sensor.INTEGRATED), false);

    motor.burnFlash();

    encoder = motor.getEncoder();
  }

  @Override
  public void set(double power) {
    motor.set(power);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }
}
