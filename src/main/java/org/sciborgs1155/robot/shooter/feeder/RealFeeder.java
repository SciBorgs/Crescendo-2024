package org.sciborgs1155.robot.shooter.feeder;

import static org.sciborgs1155.robot.Ports.Shooter.Feeder.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Feeder.*;

import java.util.Set;

import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Unit;

public class RealFeeder implements FeederIO {

  private static final Unit<Current> Amps = null;
  private final CANSparkFlex motor;

  public RealFeeder() {
    motor = new CANSparkFlex(FEEDER_SPARK, MotorType.kBrushless);

    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);
    motor.setSmartCurrentLimit((int) Math.round(CURRENT_LIMIT.in(Amps)));

    motor.burnFlash();

   SparkUtils.configureFrameStrategy(
    motor, 
    Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE),
    Set.of(Sensor.INTEGRATED),
    false
    );
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
