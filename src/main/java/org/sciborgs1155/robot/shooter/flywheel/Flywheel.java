package org.sciborgs1155.robot.shooter.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.shooter.ShooterConstants.FlywheelConstants;

public class Flywheel extends SubsystemBase implements AutoCloseable, Logged {
  private final FlywheelIO flywheel;
  private final SysIdRoutine sysIdoogabooga;

  private final PIDController flywheelPID =
      new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);

  private final SimpleMotorFeedforward flywheelFeedforward =
      new SimpleMotorFeedforward(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);

  /** Creates real or simulated flywheel based on {@link Robot#isReal()} */
  public static Flywheel create() {
    return Robot.isReal() ? new Flywheel(new RealFlywheel()) : new Flywheel(new SimFlywheel());
  }

  public Flywheel(FlywheelIO flywheel) {
    this.flywheel = flywheel;
    sysIdoogabooga = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism((v) -> flywheel.setVoltage(v.in(Volts)), null, this, "flywheel"));
  
    SmartDashboard.putData("quasistaticBack", quasistaticBack());
    SmartDashboard.putData("quasistaticForward", quasistaticForward());
    SmartDashboard.putData("dynamicBack", dynamicBack());
    SmartDashboard.putData("dynamicForward", dynamicForward());
  }

  /**
   * Run the flywheel at a specified velocity.
   *
   * @param velocity The desired velocity.
   * @return The command to set the flywheel's velocity.
   */
  public Command runFlywheel(DoubleSupplier velocity) {
    return run(() ->
            flywheel.setVoltage(
                flywheelPID.calculate(flywheel.getVelocity(), velocity.getAsDouble())
                    + flywheelFeedforward.calculate(velocity.getAsDouble())))
        .withName("running Flywheel");
  }

  public double getVelocity() {
    return flywheel.getVelocity();
  }

  public boolean atSetpoint() {
    return flywheelPID.atSetpoint();
  }

  public Command quasistaticBack() {
    return sysIdoogabooga.quasistatic(Direction.kReverse);
  }

  public Command quasistaticForward() {
    return sysIdoogabooga.quasistatic(Direction.kForward);
  }

  public Command dynamicForward() {
    return sysIdoogabooga.dynamic(Direction.kForward);
  }

  public Command dynamicBack() {
    return sysIdoogabooga.dynamic(Direction.kReverse);
  }

  @Override
  public void close() throws Exception {
    flywheel.close();
  }
}
