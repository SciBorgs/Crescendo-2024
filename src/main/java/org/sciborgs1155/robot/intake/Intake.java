package org.sciborgs1155.robot.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Consumer;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {
  private final Consumer<Double> motor;

  public Intake(Consumer<Double> motor) {
    this.motor = motor;
    setDefaultCommand(run(() -> motor.accept(0.)));
  }

  public Command intake() {
    return run(() -> motor.accept(IntakeConstants.INTAKE_SPEED));
  }

  public Command outtake() {
    return run(() -> motor.accept(-IntakeConstants.INTAKE_SPEED));
  }

  /**
   * Creates intake with a real motor. THERE MIGHT BE A RESOURCE LEAK.
   *
   * @return
   */
  public static Intake create() {
    CANSparkFlex spark =
        new CANSparkFlex(IntakeConstants.INTAKE_DEVICE_ID, MotorType.kBrushless); // RESOURCE LEAK!
    return new Intake(spark::setVoltage);
  }
}
