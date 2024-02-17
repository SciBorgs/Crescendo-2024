package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.robot.Ports;

public interface IntakeIO extends AutoCloseable {
  void setPower(double percentage);

  // boolean beamBreak();

  public static class RealIntake implements IntakeIO {
    private final CANSparkFlex spark =
        new CANSparkFlex(Ports.Intake.INTAKE_SPARK, MotorType.kBrushless);

    // private final DigitalInput beambreak = new DigitalInput(Ports.Intake.BEAMBREAK);

    public RealIntake() {
      spark.restoreFactoryDefaults();
      spark.setInverted(true);
      spark.setIdleMode(IdleMode.kBrake);
      spark.setSmartCurrentLimit((int) IntakeConstants.CURRENT_LIMIT.in(Amps));

      SparkUtils.configureNothingFrameStrategy(spark);
      spark.burnFlash();
    }

    @Override
    public void setPower(double percentage) {
      spark.set(percentage);
    }

    // @Override
    // public boolean beamBreak() {
    //   return beambreak.get();
    // }

    @Override
    public void close() {
      spark.close();
      // beambreak.close();
    }
  }

  public static class NoIntake implements IntakeIO {
    @Override
    public void setPower(double percentage) {}

    // @Override
    // public boolean beamBreak() {
    //   return false;
    // }

    @Override
    public void close() {}
  }
}
