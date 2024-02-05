package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
    encoder.setVelocityConversionFactor(VELOCITY_CONVERSION.in(MetersPerSecond));
    encoder.setPositionConversionFactor(POSITION_CONVERSION.in(Meters));

  }

  @Override
  public void setVoltage(double voltage) {
    motor.set(voltage);
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
