package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
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
import java.util.Optional;
import java.util.function.DoubleSupplier;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.commands.Shooting;

public class Shooter extends SubsystemBase implements AutoCloseable, Logged {
  private final WheelIO top;
  private final WheelIO bottom;

  private double setpoint;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  private final DoubleEntry p = Tuning.entry("/Robot/shooter/P", kP);
  private final DoubleEntry i = Tuning.entry("/Robot/shooter/I", kI);
  private final DoubleEntry d = Tuning.entry("/Robot/shooter/D", kD);

  private final PIDController topPID = new PIDController(kP, kI, kD);
  private final PIDController bottomPID = new PIDController(kP, kI, kD);

  private final SysIdRoutine sysId;

  public static Shooter create() {
    return Robot.isReal()
        ? new Shooter(new RealWheel(TOP_MOTOR), new RealWheel(BOTTOM_MOTOR))
        : new Shooter(new SimWheel(), new SimWheel());
  }

  public static Shooter none() {
    return new Shooter(new NoWheel(), new NoWheel());
  }

  public Shooter(WheelIO top, WheelIO bottom) {
    this.top = top;
    this.bottom = bottom;

    top.setInverted(true);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(10.0), Seconds.of(11)),
            new SysIdRoutine.Mechanism(v -> setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putData("shooter quasistatic backward", quasistaticBack());
    SmartDashboard.putData("shooter quasistatic forward", quasistaticForward());
    SmartDashboard.putData("shooter dynamic backward", dynamicBack());
    SmartDashboard.putData("shooter dynamic forward", dynamicForward());

    setDefaultCommand(run(() -> setSetpoint(IDLE_VELOCITY.in(RadiansPerSecond))));
  }

  public void setVoltage(double voltage) {
    top.setVoltage(voltage);
    bottom.setVoltage(voltage);
  }

  public double topVelocity() {
    return top.velocity();
  }

  public double bottomVelocity() {
    return bottom.velocity();
  }

  public void setSetpoint(double velocity) {
    double ff = feedforward.calculate(setpoint, velocity, PERIOD.in(Seconds));
    double topOut = topPID.calculate(top.velocity(), velocity);
    double bottomOut = bottomPID.calculate(bottom.velocity(), velocity);
    top.setVoltage(MathUtil.clamp(ff + topOut, -12, 12));

    bottom.setVoltage(MathUtil.clamp(ff + bottomOut, -12, 12));
    setpoint = velocity;
  }

  public boolean atSetpoint() {
    return topPID.atSetpoint() && bottomPID.atSetpoint();
  }

  public double setpoint() {
    return setpoint;
  }

  public void updatePID() {
    topPID.setPID(p.get(), i.get(), d.get());
    bottomPID.setPID(p.get(), i.get(), d.get());
  }

  public Command runShooter(DoubleSupplier velocity) {
    return run(() -> setSetpoint(velocity.getAsDouble())).withName("running shooter").asProxy();
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

  public double rotationalVelocity() {
    return top.velocity();
  }

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

  @Override
  public void periodic() {
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));

    updatePID();
  }

  @Override
  public void close() throws Exception {
    top.close();
    bottom.close();
  }
}
