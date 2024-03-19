package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.intake.IntakeConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.robot.Ports;

public class RealIntake implements IntakeIO {
  private final CANSparkFlex spark =
      new CANSparkFlex(Ports.Intake.INTAKE_SPARK, MotorType.kBrushless);

  private final DigitalInput beambreak = new DigitalInput(Ports.Intake.BEAMBREAK);

  public RealIntake() {
    SparkUtils.configureNothingFrameStrategy(spark);
    FaultLogger.check(spark); SparkUtils.setInverted(spark, true);
    FaultLogger.check(spark); spark.setIdleMode(IdleMode.kBrake);
    FaultLogger.check(spark); spark.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    FaultLogger.check(spark); 
    FaultLogger.register(spark);
  }

  @Override
  public void setPower(double percentage) {
    spark.set(percentage);
    FaultLogger.check(spark);
  }

  @Override
  @Log.NT
  public boolean beambreak() {
    return beambreak.get();
  }

  @Override
  @Log.NT
  public double current() {
    return spark.getOutputCurrent();
  }

  @Override
  public void close() {
    beambreak.close();
    spark.close();
  }
}
