package org.sciborgs1155.robot.shooter.pivot;

import static org.sciborgs1155.robot.Ports.Shooter.Pivot.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.setup.TemplateMotorSetup;

public class RealPivot implements PivotIO {
  private final CANSparkFlex lead;
  // FIX THESE NAMES!!!!!!!
  private final CANSparkFlex followOne;
  private final CANSparkFlex followTwo;
  private final CANSparkFlex followThree;
  private final DutyCycleEncoder encoder;

  public RealPivot() {
    TemplateMotorSetup setup = new TemplateMotorSetup();

    lead = new CANSparkFlex(PIVOT_SPARK_ONE, MotorType.kBrushless);
    followOne = new CANSparkFlex(PIVOT_SPARK_TWO, MotorType.kBrushless);
    followTwo = new CANSparkFlex(PIVOT_SPARK_THREE, MotorType.kBrushless);
    followThree = new CANSparkFlex(PIVOT_SPARK_FOUR, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(PIVOT_THROUGHBORE);

    setup.createMotor.createFlex(lead, CURRENT_LIMIT);
    setup.createMotor.createFlex(followOne, CURRENT_LIMIT);
    setup.createMotorInverted.createFlex(followTwo, CURRENT_LIMIT);
    setup.createMotorInverted.createFlex(followThree, CURRENT_LIMIT);

    encoder.setDistancePerRotation(CONVERSION);

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

  @Override
  public double getPosition() {
    return encoder.getAbsolutePosition();
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
