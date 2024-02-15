package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.commands.NoteVisualizer;

public class Shooter extends SubsystemBase implements AutoCloseable, Logged {
  private final ShooterIO shooter;
  private final SysIdRoutine sysId; // sysIdoogabooga

  @Log.NT private final PIDController pid = new PIDController(kP, kI, kD);

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

  /** Creates real or simulated shooter based on {@link Robot#isReal()}. */
  public static Shooter create() {
    return Robot.isReal() ? new Shooter(new RealShooter()) : new Shooter(new SimShooter());
  }

  /** Creates a fake shooter. */
  public static Shooter none() {
    return new Shooter(new NoShooter());
  }

  public Shooter(ShooterIO shooter) {
    this.shooter = shooter;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(v -> shooter.setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putData("shooter quasistatic backward", quasistaticBack());
    SmartDashboard.putData("shooter quasistatic forward", quasistaticForward());
    SmartDashboard.putData("shooter dynamic backward", dynamicBack());
    SmartDashboard.putData("shooter dynamic forward", dynamicForward());
  }

  /**
   * Run the shooter at a specified velocity.
   *
   * @param velocity The desired velocity in radians per second.
   * @return The command to set the shooter's velocity.
   */
  public Command runShooter(DoubleSupplier velocity) {
    return run(() ->
            shooter.setVoltage(
                pid.calculate(shooter.getVelocity(), velocity.getAsDouble())
                    + ff.calculate(velocity.getAsDouble())))
        .andThen(NoteVisualizer.shoot())
        .withName("running shooter");
  }

  @Log.NT
  /**
   * @return Shooter velocity in radians per second
   */
  public double getVelocity() {
    return shooter.getVelocity();
  }

  @Log.NT
  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public Command quasistaticBack() {
    return sysId.quasistatic(Direction.kReverse);
  }

  public Command quasistaticForward() {
    return sysId.quasistatic(Direction.kForward);
  }

  public Command dynamicForward() {
    return sysId.dynamic(Direction.kForward);
  }

  public Command dynamicBack() {
    return sysId.dynamic(Direction.kReverse);
  }

  @Override
  public void close() throws Exception {
    shooter.close();
  }
}
