package org.sciborgs1155.robot.shooter.flywheel;

import static org.sciborgs1155.robot.Ports.Shooter.Flywheel.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.setup.TemplateMotorSetup;

public class RealFlywheel implements FlywheelIO {
  private final CANSparkFlex topMotor;
  private final CANSparkFlex bottomMotor;
  private final RelativeEncoder encoder;

  public RealFlywheel() {
    TemplateMotorSetup setup = new TemplateMotorSetup();

    topMotor = new CANSparkFlex(LEFT_MOTOR, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(RIGHT_MOTOR, MotorType.kBrushless);
    encoder = topMotor.getEncoder();

    setup.createMotor.createFlex(topMotor, CURRENT_LIMIT);
    setup.createMotorInverted.createFlex(bottomMotor, CURRENT_LIMIT);

    encoder.setVelocityConversionFactor(VELOCITY_CONVERSION);
    encoder.setPositionConversionFactor(POSITION_CONVERSION);

    SparkUtils.configureFrameStrategy(
        topMotor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE),
        Set.of(Sensor.INTEGRATED),
        true);
    SparkUtils.configureFollowerFrameStrategy(bottomMotor);

    bottomMotor.follow(topMotor);

    topMotor.burnFlash();
    bottomMotor.burnFlash();
  }

  @Override
  public void setVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    topMotor.close();
    bottomMotor.close();
  }
}
