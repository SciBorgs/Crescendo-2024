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
    SparkUtils.configure(
        spark,
        () -> SparkUtils.configureNothingFrameStrategy(spark),
        () -> SparkUtils.setInverted(spark, true),
        () -> spark.setIdleMode(IdleMode.kBrake),
        () -> spark.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));
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
    // return false;
    return beambreak.get();
  }

  @Override
  public void close() {
    // spark.close();
    beambreak.close();
  }
}
