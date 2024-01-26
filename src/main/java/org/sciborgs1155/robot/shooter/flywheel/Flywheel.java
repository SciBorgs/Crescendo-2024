package org.sciborgs1155.robot.shooter.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.shooter.ShooterConstants.FlywheelConstants;

public class Flywheel extends SubsystemBase implements AutoCloseable {
  private final FlywheelIO flywheel;

  private final PIDController flywheelPID;
  private final SimpleMotorFeedforward flywheelFeedforward;

  /** Creates real or simulated flywheel based on {@link Robot#isReal()} */
  public static Flywheel create() {
    return Robot.isReal() ? new Flywheel(new RealFlywheel()) : new Flywheel(new SimFlywheel());
  }

  public Flywheel(FlywheelIO flywheel) {
    this.flywheel = flywheel;

    flywheelPID =
        new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
    flywheelFeedforward =
        new SimpleMotorFeedforward(
            FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);
  }

  public Command runFlywheel(DoubleSupplier velocity) {
    return run(() ->
            flywheel.setVoltage(
                flywheelPID.calculate(flywheel.getVelocity(), velocity.getAsDouble())
                    + flywheelFeedforward.calculate(velocity.getAsDouble())))
        .withName("running Flywheel");
  }

  public void runFlywheelBase(DoubleSupplier velocity) {
    flywheel.setVoltage(
        flywheelPID.calculate(flywheel.getVelocity(), velocity.getAsDouble())
            + flywheelFeedforward.calculate(velocity.getAsDouble()));
  }

  @Override
  public void close() throws Exception {
    flywheel.close();
  }
}
