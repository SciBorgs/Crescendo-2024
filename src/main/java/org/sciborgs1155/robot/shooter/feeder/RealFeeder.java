package org.sciborgs1155.robot.shooter.feeder;

import static org.sciborgs1155.robot.Ports.Shooter.Feeder.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.FeederConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
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
  public void set(double speed) {
    motor.set(speed);
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
