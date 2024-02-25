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

    for (CANSparkMax spark : List.of(lead, leftTop, rightTop, rightBottom)) {
      spark.restoreFactoryDefaults();
      spark.setCANTimeout(50);
      spark.setIdleMode(IdleMode.kBrake);
      spark.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    }

    lead.setInverted(true);
    leftTop.follow(lead, false);
    rightTop.follow(lead, true);
    rightBottom.follow(lead, true);

    encoder = lead.getAlternateEncoder(SparkUtils.THROUGHBORE_CPR);
    encoder.setInverted(true);
    encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians));
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond));
    encoder.setPosition(STARTING_ANGLE.in(Radians));

    SparkUtils.configureFrameStrategy(
        lead,
        Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
        Set.of(Sensor.ALTERNATE),
        true);
    SparkUtils.configureNothingFrameStrategy(leftTop);
    SparkUtils.configureNothingFrameStrategy(rightTop);
    SparkUtils.configureNothingFrameStrategy(rightBottom);

    for (CANSparkMax spark : List.of(lead, leftTop, rightTop, rightBottom)) {
      spark.setCANTimeout(20);
      spark.burnFlash();
    }
  }

  @Override
  public void setVoltage(double voltage) {
    lead.setVoltage(voltage);
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
