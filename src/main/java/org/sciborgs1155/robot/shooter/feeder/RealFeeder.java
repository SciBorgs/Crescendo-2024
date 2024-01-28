package org.sciborgs1155.robot.shooter.feeder;

import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Ports.Shooter.Feeder.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.FeederConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealFeeder implements FeederIO {

  private final CANSparkFlex motor;

  public RealFeeder() {

    motor = new CANSparkFlex(FEEDER_SPARK, MotorType.kBrushless);

    SparkUtils.configureSettings(false, IdleMode.kBrake, CURRENT_LIMIT, motor);

    motor.burnFlash();

    SparkUtils.configureFrameStrategy(
        motor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE),
        Set.of(Sensor.INTEGRATED),
        false);
  }

  @Override
  public void set(Measure<Voltage> voltage) {
    motor.set(voltage.in(Units.Volts));
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }

  @Override
  public Measure<Voltage> getVoltage() {
    return Volts.of(motor.getBusVoltage());
  }
}
