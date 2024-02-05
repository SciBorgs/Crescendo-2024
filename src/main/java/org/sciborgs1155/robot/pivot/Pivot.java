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
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.pivot.PivotConstants.ClimbConstants;

public class Pivot extends SubsystemBase implements AutoCloseable, Logged {
  private final PivotIO pivot;
  private final SysIdRoutine sysIdRoutine; // sysIdRoutineoogabooga

  // pivot control
  @Log.NT
  private final ProfiledPIDController pivotPID =
      new ProfiledPIDController(
          PivotConstants.kP,
          PivotConstants.kI,
          PivotConstants.kD,
          new TrapezoidProfile.Constraints(PivotConstants.MAX_VELOCITY, PivotConstants.MAX_ACCEL));

  @Log.NT
  private final ArmFeedforward pivotFeedforward =
      new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);

  // climb control
  @Log.NT
  private final ProfiledPIDController climbPID =
      new ProfiledPIDController(
          ClimbConstants.kP,
          ClimbConstants.kI,
          ClimbConstants.kD,
          new TrapezoidProfile.Constraints(ClimbConstants.MAX_VELOCITY, ClimbConstants.MAX_ACCEL));

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

    SmartDashboard.putData("pivot quasistatic forward", quasistaticForward());
    SmartDashboard.putData("pivot quasistatic backward", quasistaticBack());
    SmartDashboard.putData("pivot dynamic forward", dynamicForward());
    SmartDashboard.putData("pivot dynamic backward", dynamicBack());

    setDefaultCommand(runPivot(this::getSetpoint).repeatedly());
  }

  /**
   * Smoothly angle the pivot to a desired goal using a {@link ProfiledPIDController}.
   *
   * @param goalAngle The position to move the pivot to.
   * @return The command to set the pivot's angle.
   */
  public Command runPivot(Supplier<Rotation2d> goalAngle) {
    return runOnce(() -> pivotPID.setGoal(goalAngle.get().getRadians()))
        .andThen(
            run(() ->
                    pivot.setVoltage(
                        pivotPID.calculate(
                                pivot.getPosition().getRadians(), goalAngle.get().getRadians())
                            + pivotFeedforward.calculate(
                                pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity)))
                .until(pivotPID::atGoal)
                .withName("running pivot"))
        .asProxy();
  }

  public Command manualPivot(Supplier<Double> stickInput) {
    double velocity = stickInput.get() * PivotConstants.MAX_VELOCITY.in(RadiansPerSecond);
    double periodMovement = Constants.PERIOD.in(Seconds) * velocity;
    double draftSetpoint = periodMovement + pivot.getPosition().getRadians();
    double setpoint =
        Math.max(Math.min(MAX_ANGLE.in(Radians), draftSetpoint), MIN_ANGLE.in(Radians));
    return runPivot(() -> Rotation2d.fromRadians(setpoint));
  }

  public Command climb(Supplier<Rotation2d> goalAngle) {
    return runOnce(() -> climbPID.setGoal(goalAngle.get().getRadians()))
        .andThen(
            run(() ->
                    pivot.setVoltage(
                        climbPID.calculate(
                                pivot.getPosition().getRadians(), goalAngle.get().getRadians())
                            + climbFeedforward.calculate(
                                climbPID.getSetpoint().position, climbPID.getSetpoint().velocity)))
                .until(climbPID::atGoal)
                .withName("running climb"))
        .asProxy();
  }

  @Log.NT
  public Rotation2d getPosition() {
    return pivot.getPosition();
  }

  @Log.NT
  public boolean atGoal() {
    return pivotPID.atGoal();
  }

  // ProfilePID doesn't log this stuff
  @Log.NT
  private Rotation2d getSetpoint() {
    return Rotation2d.fromRadians(pivotPID.getSetpoint().position);
  }

  public Command quasistaticForward() {
    return sysIdRoutine
        .quasistatic(Direction.kForward)
        .until(() -> pivot.getPosition().getRadians() > MAX_ANGLE.in(Radians));
  }

  public Command quasistaticBack() {
    return sysIdRoutine
        .quasistatic(Direction.kReverse)
        .until(() -> pivot.getPosition().getRadians() > MAX_ANGLE.in(Radians));
  }

  public Command dynamicForward() {
    return sysIdRoutine
        .dynamic(Direction.kForward)
        .until(() -> pivot.getPosition().getRadians() > MAX_ANGLE.in(Radians));
  }

  public Command dynamicBack() {
    return sysIdRoutine
        .dynamic(Direction.kReverse)
        .until(() -> pivot.getPosition().getRadians() > MAX_ANGLE.in(Radians));
  }

  @Override
  public void periodic() {
    positionVisualizer.setState(pivot.getPosition());
    setpointVisualizer.setState(Rotation2d.fromRadians(pivotPID.getSetpoint().position));
  }

  @Override
  public void close() throws Exception {
    pivot.close();
    measurement.close();
    setpoint.close();
  }
}
