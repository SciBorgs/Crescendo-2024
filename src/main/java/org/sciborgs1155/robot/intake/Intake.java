package org.sciborgs1155.robot.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged, AutoCloseable {
  private final CANSparkFlex motor;

  public Intake() {
    motor = new CANSparkFlex(IntakeConstants.INTAKE_DEVICE_ID, MotorType.kBrushless);
    setDefaultCommand(run(motor::disable));
  }

  public Command intake() {
    return run(() -> motor.set(IntakeConstants.INTAKE_SPEED));
  }

  public Command outtake() {
    return run(() -> motor.set(-IntakeConstants.INTAKE_SPEED));
  }

  public void close() throws Exception {
    motor.close();
  }
}
