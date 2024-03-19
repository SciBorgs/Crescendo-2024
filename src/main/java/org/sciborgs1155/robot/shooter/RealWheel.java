package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealWheel implements WheelIO {
  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;

  public RealWheel(int id) {
    motor = new CANSparkFlex(id, MotorType.kBrushless);
    encoder = motor.getEncoder();

    SparkUtils.configure(
        motor,
        () ->
            SparkUtils.configureFrameStrategy(
                motor,
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.INTEGRATED),
                false),
        () -> motor.setIdleMode(IdleMode.kCoast),
        () -> motor.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
        () -> encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians)),
        () -> encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond)),
        () -> encoder.setAverageDepth(16),
        () -> encoder.setMeasurementPeriod(32));
  }

  @Override
  public void setInverted(boolean inverted) {
    SparkUtils.setInverted(motor, inverted);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double velocity() {
    return encoder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
