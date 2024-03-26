package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

public class Pivot extends SubsystemBase implements AutoCloseable, Logged {
  private final PivotIO hardware;
  private final SysIdRoutine sysIdRoutine;

  // Control
  @Log.NT
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL));

  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  // Visualization
  @Log.NT
  private final PivotVisualizer positionVisualizer = new PivotVisualizer(new Color8Bit(255, 0, 0));

  @Log.NT
  private final PivotVisualizer setpointVisualizer = new PivotVisualizer(new Color8Bit(0, 0, 255));

  /** Creates a real or simulated pivot based on {@link Robot#isReal()}. */
  public static Pivot create() {
    return Robot.isReal() ? new Pivot(new RealPivot()) : new Pivot(new SimPivot());
  }

  /** Creates a nonexistent pivot. */
  public static Pivot none() {
    return new Pivot(new NoPivot());
  }

  /**
   * Constructs a new pivot subsystem.
   *
   * @param pivot The hardware implementation to use.
   */
  public Pivot(PivotIO pivot) {
    this.hardware = pivot;
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(3), Seconds.of(6)),
            new SysIdRoutine.Mechanism(v -> pivot.setVoltage(v.in(Volts)), null, this));

    pid.reset(MAX_ANGLE.in(Radians));
    pid.setTolerance(POSITION_TOLERANCE.in(Radians));

    SmartDashboard.putData("pivot quasistatic forward", quasistaticForward());
    SmartDashboard.putData("pivot quasistatic backward", quasistaticBack());
    SmartDashboard.putData("pivot dynamic forward", dynamicForward());
    SmartDashboard.putData("pivot dynamic backward", dynamicBack());

    setDefaultCommand(
        run(() -> update(MAX_ANGLE.in(Radians)))
            .until(() -> pid.getGoal().position > MAX_ANGLE.in(Radians))
            .andThen(run(() -> pivot.setVoltage(0)))
            .withName("default position"));
    teleop().or(autonomous()).onTrue(Commands.runOnce(() -> pid.reset(hardware.getPosition())));
  }

  /**
   * Smoothly angle the pivot to a desired goal using a {@link ProfiledPIDController} as a proxy to
   * avoid command composition requirement conflicts.
   *
   * @param goalAngle The position to move the pivot to.
   * @return The command to set the pivot's angle.
   */
  public Command runPivot(DoubleSupplier goalAngle) {
    return run(() -> update(goalAngle.getAsDouble())).withName("go to").asProxy();
  }

  public Command runPivot(Measure<Angle> goal) {
    return runPivot(() -> goal.in(Radians));
  }

  /** Chainmaxxing fr */
  public Command lockedIn() {
    return run(() -> hardware.setVoltage(12))
        .beforeStarting(() -> hardware.setCurrentLimit(CLIMBER_CURRENT_LIMIT));
    // .until(() -> hardware.getPosition() > STARTING_ANGLE.in(Radians));
  }

  public Command manualPivot(DoubleSupplier stickInput) {
    return runPivot(
        InputStream.of(stickInput)
            .scale(MAX_VELOCITY.in(RadiansPerSecond) / 4)
            .scale(Constants.PERIOD.in(Seconds))
            .add(() -> pid.getGoal().position));
  }

  public Command setGoal(DoubleSupplier goal) {
    return runOnce(() -> pid.setGoal(goal.getAsDouble())).asProxy();
  }

  @Log.NT
  public Rotation3d rotation() {
    return new Rotation3d(0.0, hardware.getPosition(), 0.0);
  }

  @Log.NT
  public Rotation3d goal() {
    return new Rotation3d(0.0, pid.getGoal().position, 0.0);
  }

  @Log.NT
  public Rotation3d setpoint() {
    return new Rotation3d(0.0, pid.getSetpoint().position, 0.0);
  }

  @Log.NT
  public Transform3d transform() {
    return new Transform3d(AXLE_FROM_CHASSIS, rotation());
  }

  @Log.NT
  public static Transform3d transform(double position) {
    return new Transform3d(AXLE_FROM_CHASSIS, new Rotation3d(0.0, position, 0.0));
  }

  public double position() {
    return hardware.getPosition();
  }

  @Log.NT
  public boolean atGoal() {
    return pid.atGoal();
  }

  public boolean atPosition(double position) {
    return Math.abs(position - hardware.getPosition()) < POSITION_TOLERANCE.in(Radians);
  }

  public Command quasistaticForward() {
    return sysIdRoutine
        .quasistatic(Direction.kForward)
        .until(() -> hardware.getPosition() > MAX_ANGLE.in(Radians) - 0.2);
  }

  public Command quasistaticBack() {
    return sysIdRoutine
        .quasistatic(Direction.kReverse)
        .until(() -> hardware.getPosition() < MIN_ANGLE.in(Radians) + 0.2);
  }

  public Command dynamicForward() {
    return sysIdRoutine
        .dynamic(Direction.kForward)
        .until(() -> hardware.getPosition() > MAX_ANGLE.in(Radians) - 0.2);
  }

  public Command dynamicBack() {
    return sysIdRoutine
        .dynamic(Direction.kReverse)
        .until(() -> hardware.getPosition() < MIN_ANGLE.in(Radians) + 0.2);
  }

  /**
   * Smoothly angle the pivot to a desired position using a {@link ProfiledPIDController}.
   *
   * @param goalAngle The position to move the pivot to.
   */
  private void update(double goalAngle) {
    double goal = MathUtil.clamp(goalAngle, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
    var prevSetpoint = pid.getSetpoint();
    double feedback = pid.calculate(hardware.getPosition(), goal);
    double accel = (pid.getSetpoint().velocity - prevSetpoint.velocity) / PERIOD.in(Seconds);
    double feedforward =
        ff.calculate(pid.getSetpoint().position + Math.PI, pid.getSetpoint().velocity, accel);
    log("feedback output", feedback);
    log("feedforward output", feedforward);
    hardware.setVoltage(feedback + feedforward);
  }

  @Override
  public void periodic() {
    positionVisualizer.setState(hardware.getPosition());
    setpointVisualizer.setState(setpoint().getY());
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
    positionVisualizer.close();
    setpointVisualizer.close();
  }
}
