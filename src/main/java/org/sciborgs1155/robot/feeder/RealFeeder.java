package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.Ports.Feeder.*;
import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealFeeder implements FeederIO {
  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;

  // private final DigitalInput beamBreak;

  public RealFeeder() {
    motor = new CANSparkFlex(FEEDER_SPARK, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps));

    // beamBreak = new DigitalInput(BEAMBREAK);

    SparkUtils.configureFrameStrategy(
        motor, Set.of(Data.POSITION, Data.VELOCITY, Data.OUTPUT), Set.of(Sensor.INTEGRATED), false);

    encoder = motor.getEncoder();
    encoder.setVelocityConversionFactor(VELOCITY_CONVERSION.in(MetersPerSecond));
    encoder.setPositionConversionFactor(POSITION_CONVERSION.in(Meters));

    motor.burnFlash();
  }

  @Override
  public void set(double power) {
    motor.set(power);
  }

  @Override
  public boolean beambreak() {
    // TODO add beambreak
    return false;
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
