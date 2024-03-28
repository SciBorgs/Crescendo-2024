package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.lib.FaultLogger.*;
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
    // encoder = lead.getEncoder();
    encoder = lead.getAlternateEncoder(SparkUtils.THROUGHBORE_CPR);

    check(lead, lead.restoreFactoryDefaults());
    SparkUtils.configureFrameStrategy(
        lead,
        Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
        Set.of(Sensor.ALTERNATE),
        true);
    lead.setInverted(true);
    check(lead);
    check(lead, lead.setIdleMode(IdleMode.kBrake));
    check(lead, lead.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));
    check(lead, encoder.setInverted(true));
    check(lead, encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians) / 2.0));
    check(lead, encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond) / 2.0));
    check(lead, encoder.setPosition(STARTING_ANGLE.in(Radians)));
    check(lead, lead.burnFlash());

    for (CANSparkMax spark : List.of(leftTop, rightTop, rightBottom)) {
      check(spark, spark.restoreFactoryDefaults());
      check(spark, SparkUtils.configureNothingFrameStrategy(spark));
      check(spark, spark.setIdleMode(IdleMode.kBrake));
      check(spark, spark.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));
    }

    check(leftTop, leftTop.follow(lead));
    check(rightTop, rightTop.follow(lead, true));
    check(rightBottom, rightTop.follow(lead, true));

    for (CANSparkMax spark : List.of(lead, leftTop, rightTop, rightBottom)) {
      check(spark, spark.burnFlash());
      register(spark);
    }
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
    leftTop.close();
    rightTop.close();
    rightBottom.close();
  }
}
