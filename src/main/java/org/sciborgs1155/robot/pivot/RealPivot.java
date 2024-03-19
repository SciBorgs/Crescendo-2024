package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import java.util.List;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealPivot implements PivotIO {
  private final CANSparkMax lead;
  private final CANSparkMax leftTop;
  private final CANSparkMax rightTop;
  private final CANSparkMax rightBottom;
  private final RelativeEncoder encoder;

  public RealPivot() {
    lead = new CANSparkMax(SPARK_LEFT_BOTTOM, MotorType.kBrushless);
    leftTop = new CANSparkMax(SPARK_LEFT_TOP, MotorType.kBrushless);
    rightTop = new CANSparkMax(SPARK_RIGHT_TOP, MotorType.kBrushless);
    rightBottom = new CANSparkMax(SPARK_RIGHT_BOTTOM, MotorType.kBrushless);
    // encoder = lead.getEncoder();
    encoder = lead.getAlternateEncoder(SparkUtils.THROUGHBORE_CPR);

    SparkUtils.configure(
        lead,
        () ->
            SparkUtils.configureFrameStrategy(
                lead,
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.ALTERNATE),
                true),
        () -> SparkUtils.setInverted(lead, true),
        () -> lead.setIdleMode(IdleMode.kBrake),
        () -> lead.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
        () -> encoder.setInverted(true),
        () -> encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians) / 2.0), // TODO fix / 2 dumbassery
        () -> encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond) / 2.0),
        () -> encoder.setPosition(STARTING_ANGLE.in(Radians)));

    SparkUtils.configure(
        leftTop,
        () -> SparkUtils.configureNothingFrameStrategy(leftTop),
        () -> leftTop.setIdleMode(IdleMode.kBrake),
        () -> leftTop.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
        () -> leftTop.follow(lead));

    for (CANSparkMax right : List.of(rightTop, rightBottom)) {
      SparkUtils.configure(
          right,
          () -> SparkUtils.configureNothingFrameStrategy(right),
          () -> right.setIdleMode(IdleMode.kBrake),
          () -> right.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
          () -> right.follow(lead, true));
    }

    FaultLogger.register(lead);
    FaultLogger.register(leftTop);
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
    leftTop.close();
    rightTop.close();
    rightBottom.close();
  }
}
