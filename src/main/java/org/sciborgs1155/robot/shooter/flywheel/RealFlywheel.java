package org.sciborgs1155.robot.shooter.flywheel;

import static org.sciborgs1155.robot.Ports.Shooter.Flywheel.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class RealFlywheel implements FlywheelIO {
  private final CANSparkFlex flywheel = new CANSparkFlex(FLYWHEEL, MotorType.kBrushless);
  private final RelativeEncoder encoder = flywheel.getEncoder();

  public RealFlywheel() {
    flywheel.restoreFactoryDefaults();
    flywheel.setInverted(false);
    flywheel.setIdleMode(IdleMode.kBrake);
    flywheel.setSmartCurrentLimit(CURRENT_LIMIT);

    encoder.setVelocityConversionFactor(VELOCITY_CONVERSION);

    flywheel.burnFlash();
  }

  @Override
  public void setVoltage(double voltage) {
    flywheel.setVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    flywheel.close();
  }
}
