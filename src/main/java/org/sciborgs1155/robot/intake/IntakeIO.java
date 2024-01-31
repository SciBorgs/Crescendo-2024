package org.sciborgs1155.robot.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

public interface IntakeIO extends AutoCloseable {
  public void setPower(double percentage);

  public boolean getBeambreakStatus();

  public static class RealIntake implements IntakeIO {
    private final CANSparkFlex spark =
        new CANSparkFlex(IntakeConstants.INTAKE_DEVICE_ID, MotorType.kBrushless);
    private final DigitalInput beambreak = new DigitalInput(IntakeConstants.BEAMBREAK);

    @Override
    public void setPower(double percentage) {
      spark.set(percentage);
    }

    @Override
    public boolean getBeambreakStatus() {
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
    public void setPower(double percentage) {
      // RoboRioSim.getVInVoltage() * percentage;
    }

    @Override
    public boolean getBeambreakStatus() {
      return false;
    }

    @Override
    public void close() {}
  }
}
