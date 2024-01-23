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
  }

  public Command spin(boolean forward) {
    return run(() ->
            motor.set(forward ? IntakeConstants.INTAKE_SPEED : -IntakeConstants.INTAKE_SPEED))
        .finallyDo(motor::disable);
  }

  public void close() throws Exception {
    motor.close();
  }
}
