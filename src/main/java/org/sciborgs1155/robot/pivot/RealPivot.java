package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.Shooter.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
  private final DutyCycleEncoder encoder;

  public RealPivot() {

    lead = SparkUtils.createSparkMax(SPARK_LEFT_TOP, false, IdleMode.kBrake, CURRENT_LIMIT);
    leftBottom = SparkUtils.createSparkMax(SPARK_LEFT_BOTTOM, false, IdleMode.kBrake, CURRENT_LIMIT);
    rightTop = SparkUtils.createSparkMax(SPARK_RIGHT_TOP, true, IdleMode.kBrake, CURRENT_LIMIT);
    rightBottom = SparkUtils.createSparkMax(SPARK_RIGHT_BOTTOM, true, IdleMode.kBrake, CURRENT_LIMIT);

    leftBottom.follow(lead,false);
    rightTop.follow(lead,true);
    rightBottom.follow(lead,true);


    encoder = new DutyCycleEncoder(PIVOT_THROUGHBORE);

    encoder.setDistancePerRotation(POSITION_FACTOR.in(Radians));

    SparkUtils.configureFrameStrategy(
        lead, Set.of(Data.POSITION, Data.VELOCITY, Data.OUTPUT), Set.of(Sensor.INTEGRATED), true);

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

  @Log.NT
  @Override
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(encoder.getAbsolutePosition());
  }

  @Override
  public void close() throws Exception {
    lead.close();
    leftBottom.close();
    rightTop.close();
    rightBottom.close();
    encoder.close();
  }
}
