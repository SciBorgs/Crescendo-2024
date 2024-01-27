package org.sciborgs1155.robot.shooter.pivot;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.Shooter.Pivot.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealPivot implements PivotIO {
  private final CANSparkFlex lead;
  // FIX THESE NAMES!!!!!!!
  private final CANSparkFlex followOne;
  private final CANSparkFlex followTwo;
  private final CANSparkFlex followThree;
  private final DutyCycleEncoder encoder;

  public RealPivot() {

    lead = new CANSparkFlex(PIVOT_SPARK_ONE, MotorType.kBrushless);
    followOne = new CANSparkFlex(PIVOT_SPARK_TWO, MotorType.kBrushless);
    followTwo = new CANSparkFlex(PIVOT_SPARK_THREE, MotorType.kBrushless);
    followThree = new CANSparkFlex(PIVOT_SPARK_FOUR, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(PIVOT_THROUGHBORE);

    SparkUtils.configureSettings(false, IdleMode.kBrake, CURRENT_LIMIT, lead, followOne);
    SparkUtils.configureSettings(true, IdleMode.kBrake, CURRENT_LIMIT, followTwo, followThree);

    encoder.setDistancePerRotation(POSITION_FACTOR.in(Radians));

    SparkUtils.configureFrameStrategy(
        lead, Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE), Set.of(Sensor.INTEGRATED), true);

    SparkUtils.configureFollowerFrameStrategy(followOne);
    SparkUtils.configureFollowerFrameStrategy(followTwo);
    SparkUtils.configureFollowerFrameStrategy(followThree);

    followOne.follow(lead);
    followTwo.follow(lead);
    followThree.follow(lead);

    lead.burnFlash();
    followOne.burnFlash();
    followTwo.burnFlash();
    followThree.burnFlash();
  }

  @Override
  public void setVoltage(double voltage) {
    lead.setVoltage(voltage);
  }

  @Log.NT
  @Override
  public Rotation2d getPosition() {
    return new Rotation2d(encoder.getAbsolutePosition());
  }

  @Override
  public double getVelocity() {
    /* FIX THIS!!!! FIX THISISIS!!!!!!!!! DUTY CYCLE ENCODERS ARE ABSOLUTE BULLSHIT
    AND SO THEY DO NOT HAV A GET VELOCITY FUNTION !!!!!!!! GREETINGS YOUNG TRAVELER IF YOU
    FIX THIS BULLSHIT I CAN OFFER YOU THREE IRON SHOVERLS. GOOD LUCK ON YOUR ADVENTURES
    YOUNG ONE!!!!! FIX THIS !!!!!!! THE ROBOT WILL NOT DO S H I T WITHOUT THIS FUNCTION!!!
    SO FIX IT!!!!! ITS NOT SUPPOSED TO RETURN 0 BUT IT DOES GRAAHAHHHHHH GOD BLESS AMERICA
    */
    return 0;
  }

  @Override
  public void close() throws Exception {
    lead.close();
    followOne.close();
    followTwo.close();
    followThree.close();
    encoder.close();
  }
}
