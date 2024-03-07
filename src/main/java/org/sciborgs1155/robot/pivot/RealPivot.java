package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.List;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.FaultLogger;
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

    SparkUtils.configure(
        lead,
        () ->
            SparkUtils.configureFrameStrategy(
                lead,
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.ALTERNATE, Sensor.INTEGRATED),
                true),
        () -> SparkUtils.setInverted(lead, true),
        () -> lead.setIdleMode(IdleMode.kBrake),
        () -> lead.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
        // () -> encoder.setInverted(true),
        () -> encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians)),
        () -> encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond)),
        () -> encoder.setPosition(STARTING_ANGLE.in(Radians)));

    SparkUtils.configure(
        leftBottom,
        () -> SparkUtils.configureNothingFrameStrategy(leftBottom),
        () -> leftBottom.setIdleMode(IdleMode.kBrake),
        () -> leftBottom.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
        () -> leftBottom.follow(lead));

    for (CANSparkMax right : List.of(rightTop, rightBottom)) {
      SparkUtils.configure(
          right,
          () -> SparkUtils.configureNothingFrameStrategy(right),
          () -> right.setIdleMode(IdleMode.kBrake),
          () -> right.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
          () -> right.follow(lead, true));
    }

    FaultLogger.register(lead);
    FaultLogger.register(leftBottom);
    FaultLogger.register(rightBottom);
    FaultLogger.register(rightTop);
  }

  @Override
  public void setVoltage(double voltage) {
    lead.setVoltage(voltage);
    FaultLogger.check(lead);
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
