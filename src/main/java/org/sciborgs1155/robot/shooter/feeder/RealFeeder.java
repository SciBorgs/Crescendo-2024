package org.sciborgs1155.robot.shooter.feeder;

import static org.sciborgs1155.robot.Ports.Shooter.Feeder.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.FeederConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealFeeder implements FeederIO {

  private final CANSparkFlex motor;
  private final DigitalInput frontBeambreak;
  private final DigitalInput endBeambreak;

  public RealFeeder() {

    motor = new CANSparkFlex(FEEDER_SPARK, MotorType.kBrushless);

    frontBeambreak = new DigitalInput(FRONT_BEAMBREAK);
    endBeambreak = new DigitalInput(END_BEAMBREAK);

    SparkUtils.configureSettings(false, IdleMode.kBrake, CURRENT_LIMIT, motor);

    motor.burnFlash();

    SparkUtils.configureFrameStrategy(
        motor,
        Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE),
        Set.of(Sensor.INTEGRATED),
        false);
  }

  @Override
  public void set(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }

  @Override
  public boolean getFrontBeambreakValue() {
    return frontBeambreak.get();
  }

  @Override
  public boolean getEndBeambreakValue() {
    return endBeambreak.get();
  }
}
