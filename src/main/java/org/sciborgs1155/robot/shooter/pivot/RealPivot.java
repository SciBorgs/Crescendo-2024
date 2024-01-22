package org.sciborgs1155.robot.shooter.pivot;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Shooter.Pivot.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Pivot.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealPivot implements PivotIO {
  private final CANSparkMax lead;
  // FIX THESE NAMES!!!!!!!
  private final CANSparkMax follow;
  private final DutyCycleEncoder encoder;

  public RealPivot() {
    this.lead = new CANSparkMax(PIVOT_SPARK_ONE, MotorType.kBrushless);
    this.follow = new CANSparkMax(PIVOT_SPARK_TWO, MotorType.kBrushless);
    this.encoder = new DutyCycleEncoder(PIVOT_THROUGHBORE);

    lead.restoreFactoryDefaults();
    lead.setInverted(false);
    lead.setIdleMode(IdleMode.kBrake);
    lead.setSmartCurrentLimit((int) Math.round(CURRENT_LIMIT.in(Amps)));

    follow.restoreFactoryDefaults();
    follow.setInverted(false);
    follow.setIdleMode(IdleMode.kBrake);
    follow.setSmartCurrentLimit((int) Math.round(CURRENT_LIMIT.in(Amps)));

    encoder.setDistancePerRotation(CONVERSION);

    SparkUtils.configureFrameStrategy(
        lead, Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE), Set.of(Sensor.INTEGRATED), true);

    SparkUtils.configureFollowerFrameStrategy(follow);

    follow.follow(lead);

    lead.burnFlash();
    follow.burnFlash();
  }

  @Override
  public void setVoltage(double voltage) {
    lead.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return encoder.getAbsolutePosition();
  }

  @Override
  public void close() throws Exception {
    lead.close();
    follow.close();
    encoder.close();
  }
}
