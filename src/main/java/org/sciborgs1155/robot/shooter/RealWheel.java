package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.check;
import static org.sciborgs1155.lib.FaultLogger.register;
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

  public RealWheel(int id, boolean inverted) {
    motor = new CANSparkFlex(id, MotorType.kBrushless);
    encoder = motor.getEncoder();

    check(
        motor,
        SparkUtils.configureFrameStrategy(
            motor,
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.INTEGRATED),
            false));
    check(motor, motor.setIdleMode(IdleMode.kCoast));
    motor.setInverted(inverted);
    check(motor);
    check(motor, motor.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));
    check(motor, encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians)));
    check(motor, encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond)));
    check(motor, encoder.setAverageDepth(16));
    check(motor, encoder.setMeasurementPeriod(32));
    check(motor, motor.burnFlash());

    register(motor);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    check(motor);
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
