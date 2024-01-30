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
   * Creates an intake.
   *
   * @param isReal If the robot is real or not.
   * @return New intake object.
   */
  public static Intake create(boolean isReal) {
    if (isReal) {
      return new Intake(
          new Consumer<Double>() {
            private final CANSparkFlex spark =
                new CANSparkFlex(IntakeConstants.INTAKE_DEVICE_ID, MotorType.kBrushless);

            @Override
            public void accept(Double t) {
              spark.set(t);
            }
          });
    }

    return new Intake(t -> {});
  }
}
