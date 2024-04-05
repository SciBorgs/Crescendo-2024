package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.TestingUtil.assertEqualsReport;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Ports.Shooter.BOTTOM_MOTOR;
import static org.sciborgs1155.robot.Ports.Shooter.TOP_MOTOR;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.FakePDH;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.commands.Shooting;

public class Shooter extends SubsystemBase implements AutoCloseable, Logged {
  private final WheelIO top;
  private final WheelIO bottom;

  @Log.NT private double setpoint;

  private final SimpleMotorFeedforward topFeedforward =
      new SimpleMotorFeedforward(Top.kS, Top.kV, Top.kA);
  private final SimpleMotorFeedforward bottomFeedforward =
      new SimpleMotorFeedforward(Bottom.kS, Bottom.kV, Bottom.kA);

  private final DoubleEntry p = Tuning.entry("/Robot/shooter/P", kP);
  private final DoubleEntry i = Tuning.entry("/Robot/shooter/I", kI);
  private final DoubleEntry d = Tuning.entry("/Robot/shooter/D", kD);

  @Log.NT private final PIDController topPID = new PIDController(kP, kI, kD);
  @Log.NT private final PIDController bottomPID = new PIDController(kP, kI, kD);

  private final SysIdRoutine topCharacterization;
  private final SysIdRoutine bottomCharacterization;

  /** Creates real or simulated shooter based on {@link Robot#isReal()}. */
  public static Shooter create() {
    return Robot.isReal()
        ? new Shooter(new RealWheel(TOP_MOTOR, true), new RealWheel(BOTTOM_MOTOR, false))
        : new Shooter(new SimWheel(Top.kV, Top.kA), new SimWheel(Bottom.kV, Bottom.kA));
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

    topCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(10.0), Seconds.of(11)),
            new SysIdRoutine.Mechanism(
                v -> top.setVoltage(v.in(Volts)), null, this, "top shooter"));
    bottomCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(10.0), Seconds.of(11)),
            new SysIdRoutine.Mechanism(
                v -> bottom.setVoltage(v.in(Volts)), null, this, "bottom shooter"));

    SmartDashboard.putData(
        "shooter top quasistatic backward", topCharacterization.quasistatic(Direction.kReverse));
    SmartDashboard.putData(
        "shooter top quasistatic forward", topCharacterization.quasistatic(Direction.kForward));
    SmartDashboard.putData(
        "shooter top dynamic backward", topCharacterization.dynamic(Direction.kReverse));
    SmartDashboard.putData(
        "shooter top dynamic forward", topCharacterization.dynamic(Direction.kForward));

    SmartDashboard.putData(
        "shooter bottom quasistatic backward",
        bottomCharacterization.quasistatic(Direction.kReverse));
    SmartDashboard.putData(
        "shooter bottom quasistatic forward",
        bottomCharacterization.quasistatic(Direction.kForward));
    SmartDashboard.putData(
        "shooter bottom dynamic backward", bottomCharacterization.dynamic(Direction.kReverse));
    SmartDashboard.putData(
        "shooter bottom dynamic forward", bottomCharacterization.dynamic(Direction.kForward));

    // setDefaultCommand(run(() -> update(IDLE_VELOCITY.in(RadiansPerSecond))));
    // setDefaultCommand(run(() -> update(0)));
    setDefaultCommand(
        run(
            () -> {
              if (FakePDH.supplyCurrent() < Constants.BROWNOUT_CURRENT.in(Amps) - 50
                  && rotationalVelocity() < IDLE_VELOCITY.in(RadiansPerSecond)) {
                update(IDLE_VELOCITY.in(RadiansPerSecond));
              } else {
                setVoltage(0);
              }
            }));
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
    double topFF = topFeedforward.calculate(setpoint, velocity, PERIOD.in(Seconds));
    double topFB = topPID.calculate(top.velocity(), velocity);
    double bottomFF = bottomFeedforward.calculate(setpoint, velocity, PERIOD.in(Seconds));
    double bottomFB = bottomPID.calculate(bottom.velocity(), velocity);
    log("top output", topFF + topFB);
    log("bottom output", bottomFF + bottomFB);

    top.setVoltage(MathUtil.clamp(topFF + topFB, -12, 12));
    bottom.setVoltage(MathUtil.clamp(bottomFF + bottomFB, -12, 12));
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
    return run(() -> update(velocity.getAsDouble())).withName("running shooter");
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

  public Command goToTest(Measure<Velocity<Angle>> goal) {
    return runShooter(goal.in(RadiansPerSecond))
        .withTimeout(3)
        .finallyDo(
            () ->
                assertEqualsReport(
                    "Shooter Syst Check Speed",
                    goal.in(RadiansPerSecond),
                    rotationalVelocity(),
                    10));
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
