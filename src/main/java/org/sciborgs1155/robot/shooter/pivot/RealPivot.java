package org.sciborgs1155.robot.shooter.pivot;

import static org.sciborgs1155.robot.Ports.Shooter.Pivot.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Pivot.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class RealPivot implements PivotIO {
  private final CANSparkMax motor;
  // FIX THESE NAMES!!!!!!!
  private final CANSparkMax motorTwo;
  private final DutyCycleEncoder encoder;

  public RealPivot() {
    this.motor = new CANSparkMax(PIVOT_SPARK_ONE, MotorType.kBrushless);
    this.motorTwo = new CANSparkMax(PIVOT_SPARK_TWO, MotorType.kBrushless);
    this.encoder = new DutyCycleEncoder(PIVOT_THROUGHBORE);

    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(CURRENT_LIMIT);

    motorTwo.restoreFactoryDefaults();
    motorTwo.setInverted(false);
    motorTwo.setIdleMode(IdleMode.kBrake);
    motorTwo.setSmartCurrentLimit(CURRENT_LIMIT);

    encoder.setDistancePerRotation(CONVERSION);

    motorTwo.follow(motor);

    motor.burnFlash();
    motorTwo.burnFlash();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return encoder.getAbsolutePosition();
  }

  @Override
  public void close() throws Exception {
    motor.close();
    motorTwo.close();
    encoder.close();
  }
}
