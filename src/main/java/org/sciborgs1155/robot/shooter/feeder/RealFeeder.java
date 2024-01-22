package org.sciborgs1155.robot.shooter.feeder;

import static org.sciborgs1155.robot.Ports.Shooter.Feeder.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Feeder.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.setup.TemplateMotorSetup;

public class RealFeeder implements FeederIO {

  private final CANSparkFlex motor;

  public RealFeeder() {
    TemplateMotorSetup setup = new TemplateMotorSetup();

    motor = new CANSparkFlex(FEEDER_SPARK, MotorType.kBrushless);

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
}
