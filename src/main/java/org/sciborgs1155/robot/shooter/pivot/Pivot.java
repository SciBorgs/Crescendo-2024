package org.sciborgs1155.robot.shooter.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants;
import org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants.ClimbConstants;

public class Pivot extends SubsystemBase implements AutoCloseable {
  private final PivotIO pivot;

  // pivot control
  private final ProfiledPIDController pivotPID;
  private final ArmFeedforward pivotFeedforward;

  // climb control
  private final ProfiledPIDController climbPID;
  private final ArmFeedforward climbFeedforward;

  // visualization
  @Log.NT final Mechanism2d measurement;
  @Log.NT final Mechanism2d setpoint;

  private final PivotVisualizer positionVisualizer;
  private final PivotVisualizer setpointVisualizer;

  /** Creates a real or simulated pivot based on {@link Robot#isReal()} */
  public static Pivot create() {
    return Robot.isReal() ? new Pivot(new RealPivot()) : new Pivot(new SimPivot());
  }

  public Pivot(PivotIO pivot) {
    this.pivot = pivot;

    pivotPID =
        new ProfiledPIDController(
            PivotConstants.kP,
            PivotConstants.kI,
            PivotConstants.kD,
            new TrapezoidProfile.Constraints(
                PivotConstants.MAX_VELOCITY, PivotConstants.MAX_ACCEL));
    pivotFeedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);

    climbPID =
        new ProfiledPIDController(
            ClimbConstants.kP,
            ClimbConstants.kI,
            ClimbConstants.kD,
            new TrapezoidProfile.Constraints(
                ClimbConstants.MAX_VELOCITY, ClimbConstants.MAX_ACCEL));
    climbFeedforward = new ArmFeedforward(ClimbConstants.kS, ClimbConstants.kG, ClimbConstants.kV);

    measurement = new Mechanism2d(3, 4);
    setpoint = new Mechanism2d(3, 4);
    positionVisualizer = new PivotVisualizer(measurement, new Color8Bit(255, 0, 0));
    setpointVisualizer = new PivotVisualizer(setpoint, new Color8Bit(0, 0, 255));
  }

  /**
   * Smoothly angle the pivot to a desired goal using a {@link ProfiledPIDController}.
   *
   * @param goalAngle The position to move the pivot to.
   * @return The command to set the pivot's angle.
   */
  public Command runPivot(Supplier<Measure<Angle>> goalAngle) {
    return runOnce(() -> pivotPID.setGoal(goalAngle.get().in(Units.Radians)))
        .andThen(
            run(() ->
                    pivot.setVoltage(
                        pivotPID.calculate(pivot.getPosition())
                            + pivotFeedforward.calculate(
                                pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity)))
                .withName("running Pivot"));
  }

  // public void runPivotBase(Supplier<Measure<Angle>> goalAngle) {
  //   pivotPID.setGoal(goalAngle.get().in(Units.Radians));
  //   pivot.setVoltage(
  //       pivotPID.calculate(pivot.getPosition())
  //           + pivotFeedforward.calculate(
  //               pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity));
  // }

  public Command climb(Supplier<Measure<Angle>> goalAngle) {
    return runOnce(() -> climbPID.setGoal(goalAngle.get().in(Radians)))
        .andThen(
            run(() ->
                    pivot.setVoltage(
                        climbPID.calculate(pivot.getPosition())
                            + climbFeedforward.calculate(
                                climbPID.getSetpoint().position, climbPID.getSetpoint().velocity)))
                .withName("running Climb"));
  }

  // public void runClimbBase(Supplier<Measure<Angle>> goalAngle) {
  //   climbPID.setGoal(goalAngle.get().in(Radians));
  //   pivot.setVoltage(
  //       climbPID.calculate(pivot.getPosition())
  //           + climbFeedforward.calculate(
  //               climbPID.getSetpoint().position, climbPID.getSetpoint().velocity));
  // }

  public double getPosition() {
    return pivot.getPosition();
  }

  // ProfilePID doesn't log this stuff
  @Log.NT
  public Measure<Angle> getSetpointRadians() {
    return Radians.of(pivotPID.getSetpoint().position);
  }

  @Override
  public void periodic() {
    positionVisualizer.setState(Degrees.convertFrom(pivot.getPosition(), Radians));
    setpointVisualizer.setState(Degrees.convertFrom(pivotPID.getSetpoint().position, Radians));
  }

  @Override
  public void close() throws Exception {
    pivot.close();
  }
}
