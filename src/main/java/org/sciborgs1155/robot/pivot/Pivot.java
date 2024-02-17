package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.pivot.PivotConstants.ClimbConstants;

public class Pivot extends SubsystemBase implements AutoCloseable, Logged {
  private final PivotIO hardware;
  private final SysIdRoutine sysIdRoutine;

  // pivot control
  @Log.NT
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL));

  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV);

  // visualization
  @Log.NT final Mechanism2d measurement = new Mechanism2d(2, 2);
  @Log.NT final Mechanism2d setpoint = new Mechanism2d(2, 2);

  private final PivotVisualizer positionVisualizer =
      new PivotVisualizer(measurement, new Color8Bit(255, 0, 0));
  private final PivotVisualizer setpointVisualizer =
      new PivotVisualizer(setpoint, new Color8Bit(0, 0, 255));

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

    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    pid.setGoal(new State(STARTING_ANGLE.getRadians(), 0));

    SmartDashboard.putData("pivot quasistatic forward", quasistaticForward());
    SmartDashboard.putData("pivot quasistatic backward", quasistaticBack());
    SmartDashboard.putData("pivot dynamic forward", dynamicForward());
    SmartDashboard.putData("pivot dynamic backward", dynamicBack());

    setDefaultCommand(run(() -> update(STARTING_ANGLE.getRadians())).withName("default position"));
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

  /**
   * Smoothly angle the pivot to a desired angle using a separately tuned {@link
   * ProfiledPIDController} for climbing.
   *
   * @return The command to set the pivot's angle.
   */
  public Command climb(double goalAngle) {
    return runOnce(() -> pid.setPID(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD))
        .andThen(() -> update(goalAngle))
        .finallyDo(() -> pid.setPID(kP, kI, kD));
  }

  public Command manualPivot(InputStream stickInput) {
    return runPivot(
        () -> {
          double velocity = stickInput.get() * MAX_VELOCITY.in(RadiansPerSecond);
          double periodMovement = Constants.PERIOD.in(Seconds) * velocity;
          double setpoint = periodMovement + pid.getSetpoint().position;
          return setpoint;
        });
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
  public Pose3d pose() {
    return new Pose3d(OFFSET, rotation());
  }

  @Log.NT
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(hardware.getPosition());
  }

  @Log.NT
  public boolean atGoal() {
    return pid.atGoal();
  }

  public Command quasistaticForward() {
    return sysIdRoutine
        .quasistatic(Direction.kForward)
        .until(() -> hardware.getPosition() > MAX_ANGLE.getRadians() - 0.2);
  }

  public Command quasistaticBack() {
    return sysIdRoutine
        .quasistatic(Direction.kReverse)
        .until(() -> hardware.getPosition() < MIN_ANGLE.getRadians() + 0.2);
  }

  public Command dynamicForward() {
    return sysIdRoutine
        .dynamic(Direction.kForward)
        .until(() -> hardware.getPosition() > MAX_ANGLE.getRadians() - 0.2);
  }

  public Command dynamicBack() {
    return sysIdRoutine
        .dynamic(Direction.kReverse)
        .until(() -> hardware.getPosition() < MIN_ANGLE.getRadians() + 0.2);
  }

  /**
   * Smoothly angle the pivot to a desired position using a {@link ProfiledPIDController}.
   *
   * @param goalAngle The position to move the pivot to.
   */
  private void update(double goalAngle) {
    double goal = MathUtil.clamp(goalAngle, MIN_ANGLE.getRadians(), MAX_ANGLE.getRadians());
    double feedback = pid.calculate(hardware.getPosition(), goal);
    double feedforward =
        ff.calculate(pid.getSetpoint().position + Math.PI, pid.getSetpoint().velocity);
    log("feedback output", feedback);
    log("feedforward output", feedforward);
    hardware.setVoltage(feedback + feedforward);
  }

  @Override
  public void periodic() {
    positionVisualizer.setState(Rotation2d.fromRadians(hardware.getPosition()));
    setpointVisualizer.setState(Rotation2d.fromRadians(pid.getSetpoint().position));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
    measurement.close();
    setpoint.close();
  }
}
