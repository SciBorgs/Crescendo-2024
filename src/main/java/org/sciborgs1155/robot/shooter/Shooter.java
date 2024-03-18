package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.commands.Shooting;

public class Shooter extends SubsystemBase implements AutoCloseable, Logged {
  private final ShooterIO shooter;
  private final SysIdRoutine sysId; // sysIdoogabooga

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
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(10.0), Seconds.of(11)),
            new SysIdRoutine.Mechanism(v -> shooter.setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putData("shooter quasistatic backward", quasistaticBack());
    SmartDashboard.putData("shooter quasistatic forward", quasistaticForward());
    SmartDashboard.putData("shooter dynamic backward", dynamicBack());
    SmartDashboard.putData("shooter dynamic forward", dynamicForward());

    setDefaultCommand(run(() -> shooter.setSetpoint(IDLE_VELOCITY.in(RadiansPerSecond))));
  }

  /**
   * Run the shooter at a specified velocity.
   *
   * @param velocity The desired velocity in radians per second.
   * @return The command to set the shooter's velocity.
   */
  public Command runShooter(DoubleSupplier velocity) {
    return run(() -> shooter.setSetpoint(velocity.getAsDouble()))
        .withName("running shooter")
        .asProxy();
  }

  // public Command manualShooter(DoubleSupplier stickInput) {
  //   return runShooter(
  //       InputStream.of(stickInput)
  //           .scale(10)
  //           .scale(Constants.PERIOD.in(Seconds))
  //           .add(pid::getSetpoint));
  // }

  public Command runShooter(double velocity) {
    return runShooter(() -> velocity);
  }

  public Command ejectStuck(double velocity) {
    return runShooter(velocity);
  }

  // public Command setSetpoint(DoubleSupplier velocity) {
  //   return runOnce(() -> pid.setSetpoint(velocity.getAsDouble())).asProxy();
  // }

  // private void update(double setpointVelocity) {
  //   double feedback = pid.calculate(shooter.topVelocity(), setpointVelocity);
  //   double feedforward = ff.calculate(setpointVelocity);
  //   log("feedback output", feedback);
  //   log("feedforward output", feedforward);
  //   shooter.setVoltage(MathUtil.clamp(feedback + feedforward, -12, 12));
  // }

  /**
   * @return Shooter velocity in radians per second
   */
  @Log.NT
  public double rotationalVelocity() {
    return shooter.topVelocity();
  }

  @Log.NT
  public double tangentialVelocity() {
    return Shooting.flywheelToNoteSpeed(rotationalVelocity());
  }

  @Log.NT
  public boolean atSetpoint() {
    return shooter.atSetpoint();
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
  public void periodic() {
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    shooter.close();
  }
}
