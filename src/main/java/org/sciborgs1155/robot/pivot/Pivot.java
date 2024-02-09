package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.pivot.PivotConstants.ClimbConstants;

public class Pivot extends SubsystemBase implements AutoCloseable, Logged {
  private final PivotIO pivot;
  private final SysIdRoutine sysIdRoutine; // sysIdRoutineoogabooga

  // pivot control
  @Log.NT
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          PivotConstants.kP,
          PivotConstants.kI,
          PivotConstants.kD,
          new TrapezoidProfile.Constraints(PivotConstants.MAX_VELOCITY, PivotConstants.MAX_ACCEL));

  private final ArmFeedforward pivotFeedforward =
      new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);

  private final ArmFeedforward climbFeedforward =
      new ArmFeedforward(ClimbConstants.kS, ClimbConstants.kG, ClimbConstants.kV);

  // visualization
  @Log.NT final Mechanism2d measurement = new Mechanism2d(3, 4);
  @Log.NT final Mechanism2d setpoint = new Mechanism2d(3, 4);

  private final PivotVisualizer positionVisualizer =
      new PivotVisualizer(measurement, new Color8Bit(255, 0, 0));
  private final PivotVisualizer setpointVisualizer =
      new PivotVisualizer(setpoint, new Color8Bit(0, 0, 255));

  /** Creates a real or simulated pivot based on {@link Robot#isReal()} */
  public static Pivot create() {
    return Robot.isReal() ? new Pivot(new RealPivot()) : new Pivot(new SimPivot());
  }

  public Pivot(PivotIO pivot) {
    this.pivot = pivot;
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                v -> pivot.setVoltage(v.in(Volts)),
                log -> {
                  log.motor("pivot")
                      .angularPosition(Radians.of(pivot.getPosition().getRadians()))
                      .angularVelocity(RadiansPerSecond.of(pivot.getVelocity()));
                },
                this,
                "pivot"));

    pid.setTolerance(POSITION_TOLERANCE.in(Radians));

    SmartDashboard.putData("pivot quasistatic forward", quasistaticForward());
    SmartDashboard.putData("pivot quasistatic backward", quasistaticBack());
    SmartDashboard.putData("pivot dynamic forward", dynamicForward());
    SmartDashboard.putData("pivot dynamic backward", dynamicBack());

    setDefaultCommand(run(this::update).repeatedly());
  }

  /**
   * Smoothly angle the pivot to a desired goal using a {@link ProfiledPIDController}.
   *
   * @param goalAngle The position to move the pivot to.
   * @return The command to set the pivot's angle.
   */
  public Command runPivot(Supplier<Rotation2d> goalAngle) {
    return run(() -> {
          double feedback =
              pid.calculate(pivot.getPosition().getRadians(), goalAngle.get().getRadians());
          double feedforward =
              pivotFeedforward.calculate( // add pi to measurement to account for alternate angle
                  pid.getSetpoint().position + Math.PI, pid.getSetpoint().velocity);
          pivot.setVoltage(feedback + feedforward);
        })
        .until(pid::atGoal)
        .withName("running pivot")
        .asProxy();
  }

  /**
   * Smoothly angle the pivot to its starting position using a {@link ProfiledPIDController}.
   *
   * <p>Sole use case as the pivot's default command due to proxying not directly exposing subsystem
   * requirements.
   *
   * @return The command to set the pivot's angle.
   */
  private Command update() {
    return run(() -> {
          double feedback =
              pid.calculate(pivot.getPosition().getRadians(), STARTING_ANGLE.getRadians());
          double feedforward =
              pivotFeedforward.calculate( // add pi to measurement to account for alternate angle
                  pid.getSetpoint().position + Math.PI, pid.getSetpoint().velocity);
          pivot.setVoltage(feedback + feedforward);
        })
        .until(pid::atGoal)
        .withName("running pivot to start");
  }

  /**
   * Smoothly angle the pivot to its starting position (from an extended, preconfigured position)
   * using a separately tuned {@link ProfiledPIDController} for climbing.
   *
   * @return The command to set the pivot's angle.
   */
  public Command climb() {
    return runOnce(() -> pid.setPID(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD))
        .andThen(this::update)
        .finallyDo(() -> pid.setPID(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD));
  }

  public Command manualPivot(InputStream stickInput) {
    return runPivot(
        () -> {
          double velocity = stickInput.get() * PivotConstants.MAX_VELOCITY.in(RadiansPerSecond);
          double periodMovement = Constants.PERIOD.in(Seconds) * velocity;
          double draftSetpoint = periodMovement + pivot.getPosition().getRadians();
          double setpoint =
              Math.max(Math.min(MAX_ANGLE.getRadians(), draftSetpoint), MIN_ANGLE.getRadians());
          return Rotation2d.fromRadians(setpoint);
        });
  }

  @Log.NT
  public Rotation2d getPosition() {
    return pivot.getPosition();
  }

  @Log.NT
  public Rotation2d getGoal() {
    return Rotation2d.fromRadians(pid.getGoal().position);
  }

  @Log.NT
  public Rotation2d getSetpoint() {
    return Rotation2d.fromRadians(pid.getSetpoint().position);
  }

  @Log.NT
  public boolean atGoal() {
    return pid.atGoal();
  }

  public Command quasistaticForward() {
    return sysIdRoutine
        .quasistatic(Direction.kForward)
        .until(() -> pivot.getPosition().getRadians() > MAX_ANGLE.getRadians());
  }

  public Command quasistaticBack() {
    return sysIdRoutine
        .quasistatic(Direction.kReverse)
        .until(() -> pivot.getPosition().getRadians() < MIN_ANGLE.getRadians());
  }

  public Command dynamicForward() {
    return sysIdRoutine
        .dynamic(Direction.kForward)
        .until(() -> pivot.getPosition().getRadians() > MAX_ANGLE.getRadians());
  }

  public Command dynamicBack() {
    return sysIdRoutine
        .dynamic(Direction.kReverse)
        .until(() -> pivot.getPosition().getRadians() < MIN_ANGLE.getRadians());
  }

  @Override
  public void periodic() {
    positionVisualizer.setState(getPosition());
    setpointVisualizer.setState(getSetpoint());
  }

  @Override
  public void close() throws Exception {
    pivot.close();
    measurement.close();
    setpoint.close();
  }
}
