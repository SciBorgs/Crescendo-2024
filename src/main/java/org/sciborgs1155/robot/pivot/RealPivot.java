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
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import java.util.Set;
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

    for (CANSparkMax spark : List.of(lead, leftBottom, rightTop, rightBottom)) {
      spark.restoreFactoryDefaults();
      spark.setIdleMode(IdleMode.kBrake);
      spark.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    }

    lead.setInverted(false);
    leftBottom.follow(lead, false);
    rightTop.follow(lead, true);
    rightBottom.follow(lead, true);

    encoder = lead.getAlternateEncoder(SparkUtils.THROUGHBORE_CPR);

    encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians));
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond));

    SparkUtils.configureFrameStrategy(
        lead, Set.of(Data.POSITION, Data.VELOCITY, Data.OUTPUT), Set.of(Sensor.QUADRATURE), true);
    SparkUtils.configureNothingFrameStrategy(leftBottom);
    SparkUtils.configureNothingFrameStrategy(rightTop);
    SparkUtils.configureNothingFrameStrategy(rightBottom);

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
