package org.sciborgs1155.robot.shooter.feeder;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.Ports.Shooter.Feeder.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.FeederConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

import com.revrobotics.RelativeEncoder;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.setup.TemplateMotorSetup;

public class RealFeeder implements FeederIO {

  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;

  public RealFeeder() {
    TemplateMotorSetup setup = new TemplateMotorSetup();

    motor = new CANSparkFlex(FEEDER_SPARK, MotorType.kBrushless);
    encoder = motor.getEncoder();

    setup.createMotor.createFlex(motor, CURRENT_LIMIT);

    motor.burnFlash();

    SparkUtils.configureFrameStrategy(
        motor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE),
        Set.of(Sensor.INTEGRATED),
        false);
  }

  @Override
  public void set(Measure<Velocity<Distance>> speed) {
    motor.set(speed.in(Units.MetersPerSecond));
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }

  @Override
  public Measure<Velocity<Distance>> getVelocity() {
    return MetersPerSecond.of(encoder.getVelocity());
  }
}
