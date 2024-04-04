package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming;
import static org.sciborgs1155.lib.TestingUtil.assertEqualsReport;
import static org.sciborgs1155.lib.TestingUtil.parameterizedSystemsCheck;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Ports.Shooter.BOTTOM_MOTOR;
import static org.sciborgs1155.robot.Ports.Shooter.TOP_MOTOR;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.commands.Shooting;

public class Shooter extends SubsystemBase implements AutoCloseable, Logged {
  private final WheelIO top;
  private final WheelIO bottom;

  @Log.NT private double setpoint;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  private final DoubleEntry p = Tuning.entry("/Robot/shooter/P", kP);
  private final DoubleEntry i = Tuning.entry("/Robot/shooter/I", kI);
  private final DoubleEntry d = Tuning.entry("/Robot/shooter/D", kD);

  @Log.NT private final PIDController topPID = new PIDController(kP, kI, kD);
  @Log.NT private final PIDController bottomPID = new PIDController(kP, kI, kD);

  private final SysIdRoutine sysId;

  /** Creates real or simulated shooter based on {@link Robot#isReal()}. */
  public static Shooter create() {
    return Robot.isReal()
        ? new Shooter(new RealWheel(TOP_MOTOR, true), new RealWheel(BOTTOM_MOTOR, false))
        : new Shooter(new SimWheel(), new SimWheel());
  }

  /** Creates a fake shooter. */
  public static Shooter none() {
    return new Shooter(new NoWheel(), new NoWheel());
  }

  public Shooter(WheelIO top, WheelIO bottom) {
    this.top = top;
    this.bottom = bottom;

    topPID.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));
    bottomPID.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(10.0), Seconds.of(11)),
            new SysIdRoutine.Mechanism(v -> setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putData("shooter quasistatic backward", quasistaticBack());
    SmartDashboard.putData("shooter quasistatic forward", quasistaticForward());
    SmartDashboard.putData("shooter dynamic backward", dynamicBack());
    SmartDashboard.putData("shooter dynamic forward", dynamicForward());

    setDefaultCommand(run(() -> update(IDLE_VELOCITY.in(RadiansPerSecond))));
  }

  public void setVoltage(double voltage) {
    top.setVoltage(voltage);
    bottom.setVoltage(voltage);
  }

  @Log.NT
  public double topVelocity() {
    return top.velocity();
  }

  @Log.NT
  public double bottomVelocity() {
    return bottom.velocity();
  }

  public void update(double velocity) {
    double ff = feedforward.calculate(setpoint, velocity, PERIOD.in(Seconds));
    double topOut = topPID.calculate(top.velocity(), velocity);
    double bottomOut = bottomPID.calculate(bottom.velocity(), velocity);
    log("top output", topOut);
    log("bottom output", bottomOut);

    top.setVoltage(MathUtil.clamp(ff + topOut, -12, 12));
    bottom.setVoltage(MathUtil.clamp(ff + bottomOut, -12, 12));
    setpoint = velocity;
  }

  @Log.NT
  public boolean atSetpoint() {
    return topPID.atSetpoint() && bottomPID.atSetpoint();
  }

  public double setpoint() {
    return setpoint;
  }

  /**
   * Run the shooter at a specified velocity.
   *
   * @param velocity The desired velocity in radians per second.
   * @return The command to set the shooter's velocity.
   */
  public Command runShooter(DoubleSupplier velocity) {
    return run(() -> update(velocity.getAsDouble())).withName("running shooter"); // .asProxy();
  }

  public Command manualShooter(DoubleSupplier stickInput) {
    return runShooter(
        InputStream.of(stickInput).scale(10).scale(PERIOD.in(Seconds)).add(this::setpoint));
  }

  public Command runShooter(double velocity) {
    return runShooter(() -> velocity);
  }

  public Command ejectStuck(double velocity) {
    return runShooter(velocity);
  }

  /**
   * @return Average shooter velocity in radians per second
   */
  @Log.NT
  public double rotationalVelocity() {
    return (topVelocity() + bottomVelocity()) / 2.0;
  }

  @Log.NT
  public double tangentialVelocity() {
    return Shooting.flywheelToNoteSpeed(rotationalVelocity());
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

  public Command systemsCheck() {
    Function<Double, Command> check =
        vel ->
            runShooter(vel)
                .beforeStarting(() -> update(vel), this)
                .until(this::atSetpoint)
                .withTimeout(3)
                .finallyDo(
                    () ->
                        assertEqualsReport(
                            "Shooter Syst Check Vel", vel, rotationalVelocity(), 10));
    return parameterizedSystemsCheck(check, List.of(100., 300., 400., 600.))
        .withInterruptBehavior(kCancelIncoming);
  }

  @Override
  public void periodic() {
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
    topPID.setPID(p.get(), i.get(), d.get());
    bottomPID.setPID(p.get(), i.get(), d.get());
  }

  @Override
  public void close() throws Exception {
    top.close();
    bottom.close();
  }
}
