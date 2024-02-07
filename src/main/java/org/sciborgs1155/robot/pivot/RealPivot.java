package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealPivot implements PivotIO {
  private final CANSparkMax lead;
  private final CANSparkMax leftBottom;
  private final CANSparkMax rightTop;
  private final CANSparkMax rightBottom;
  private final SparkAbsoluteEncoder encoder;

  public RealPivot() {
    lead = SparkUtils.createSparkMax(SPARK_LEFT_TOP, false, IdleMode.kBrake, CURRENT_LIMIT);
    leftBottom =
        SparkUtils.createSparkMax(SPARK_LEFT_BOTTOM, false, IdleMode.kBrake, CURRENT_LIMIT);
    rightTop = SparkUtils.createSparkMax(SPARK_RIGHT_TOP, true, IdleMode.kBrake, CURRENT_LIMIT);
    rightBottom =
        SparkUtils.createSparkMax(SPARK_RIGHT_BOTTOM, true, IdleMode.kBrake, CURRENT_LIMIT);

    leftBottom.follow(lead, false);
    rightTop.follow(lead, true);
    rightBottom.follow(lead, true);

    encoder = lead.getAbsoluteEncoder(Type.kDutyCycle);

    encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians));
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond));

    SparkUtils.configureFrameStrategy(
        lead, Set.of(Data.POSITION, Data.VELOCITY, Data.OUTPUT), Set.of(Sensor.DUTY_CYCLE), true);

    SparkUtils.configureFollowerFrameStrategy(leftBottom);
    SparkUtils.configureFollowerFrameStrategy(rightTop);
    SparkUtils.configureFollowerFrameStrategy(rightBottom);

    lead.burnFlash();
    leftBottom.burnFlash();
    rightTop.burnFlash();
    rightBottom.burnFlash();
  }

  @Override
  public void setVoltage(double voltage) {
    lead.setVoltage(voltage);
  }

  @Override
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(encoder.getPosition());
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
