package org.sciborgs1155.robot.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import org.sciborgs1155.robot.Ports;

public interface IntakeIO extends AutoCloseable {
  void setPower(double percentage);

  boolean beamBreak();

  public static class RealIntake implements IntakeIO {
    private final CANSparkFlex spark =
        new CANSparkFlex(Ports.Intake.INTAKE_DEVICE_ID, MotorType.kBrushless);
    private final DigitalInput beambreak = new DigitalInput(Ports.Intake.BEAMBREAK);

    @Override
    public void setPower(double percentage) {
      spark.set(percentage);
    }

    @Override
    public boolean beamBreak() {
      return beambreak.get();
    }

    @Override
    public void close() {
      spark.close();
      beambreak.close();
    }
  }

  public static class NoIntake implements IntakeIO {
    @Override
    public void setPower(double percentage) {}

    @Override
    public boolean beamBreak() {
      return false;
    }

    @Override
    public void close() {}
  }
}
