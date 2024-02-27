package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Feeder.*;
import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;

public class RealFeeder implements FeederIO {
  private final CANSparkFlex motor;

  private final DigitalInput beambreak;

  public RealFeeder() {
    motor = new CANSparkFlex(FEEDER_SPARK, MotorType.kBrushless);

    SparkUtils.configure(
        motor,
        () -> SparkUtils.configureNothingFrameStrategy(motor),
        () -> motor.setIdleMode(IdleMode.kBrake),
        () -> motor.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));

    beambreak = new DigitalInput(BEAMBREAK);

    FaultLogger.register(motor);
  }

  @Override
  public void set(double power) {
    motor.set(power);
    FaultLogger.check(motor);
  }

  @Override
  @Log.NT
  public boolean beambreak() {
    return beambreak.get();
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
