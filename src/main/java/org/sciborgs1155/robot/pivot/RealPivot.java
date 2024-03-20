package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import static org.sciborgs1155.lib.FaultLogger.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.List;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealPivot implements PivotIO {
  private final CANSparkMax lead;
  private final CANSparkMax leftBottom;
  private final CANSparkMax rightTop;
  private final CANSparkMax rightBottom;
  private final RelativeEncoder encoder;

  public RealPivot() {
    lead = new CANSparkMax(SPARK_LEFT_TOP, MotorType.kBrushless);
    leftBottom = new CANSparkMax(SPARK_LEFT_BOTTOM, MotorType.kBrushless);
    rightTop = new CANSparkMax(SPARK_RIGHT_TOP, MotorType.kBrushless);
    rightBottom = new CANSparkMax(SPARK_RIGHT_BOTTOM, MotorType.kBrushless);
    encoder = lead.getEncoder();
    // encoder = lead.getAlternateEncoder(SparkUtils.THROUGHBORE_CPR);

    check(lead, lead.restoreFactoryDefaults());
    check(
        lead,
        SparkUtils.configureFrameStrategy(
            lead,
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.ALTERNATE, Sensor.INTEGRATED),
            true));
    lead.setInverted(true);
    check(lead);
    check(lead, lead.setIdleMode(IdleMode.kBrake));
    check(lead, lead.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));
    // check(lead, encoder.setInverted(true));
    check(lead, encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians)));
    check(lead, encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond)));
    check(lead, encoder.setPosition(STARTING_ANGLE.in(Radians)));
    check(lead, lead.burnFlash());

    check(leftBottom, leftBottom.restoreFactoryDefaults());
    check(leftBottom, SparkUtils.configureNothingFrameStrategy(leftBottom));
    check(leftBottom, leftBottom.setIdleMode(IdleMode.kBrake));
    check(leftBottom, leftBottom.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));
    check(leftBottom, leftBottom.follow(lead));
    check(leftBottom, leftBottom.burnFlash());

    for (CANSparkMax right : List.of(rightTop, rightBottom)) {
      check(right, right.restoreFactoryDefaults());
      check(right, SparkUtils.configureNothingFrameStrategy(right));
      check(right, right.setIdleMode(IdleMode.kBrake));
      check(right, right.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));
      check(right, right.follow(lead, true));
      check(right, right.burnFlash());
    }

    register(lead);
    register(leftBottom);
    register(rightBottom);
    register(rightTop);
  }

  @Override
  public void setVoltage(double voltage) {
    lead.setVoltage(voltage);
    check(lead);
  }

  @Override
  @Log.NT
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    lead.close();
    leftBottom.close();
    rightTop.close();
    rightBottom.close();
  }
}
