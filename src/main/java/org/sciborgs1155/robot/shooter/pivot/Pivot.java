package org.sciborgs1155.robot.shooter.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants;
import org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants.ClimbConstants;

public class Pivot extends SubsystemBase implements AutoCloseable, Logged {
  @Log.NT private final PivotIO pivot;

  // pivot control
  public final ProfiledPIDController pivotPID =
      new ProfiledPIDController(
          PivotConstants.kP,
          PivotConstants.kI,
          PivotConstants.kD,
          new TrapezoidProfile.Constraints(PivotConstants.MAX_VELOCITY, PivotConstants.MAX_ACCEL));
  private final ArmFeedforward pivotFeedforward =
      new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);

  // climb control
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
                        pivotPID.calculate(pivot.getPosition().getRadians())
                            + pivotFeedforward.calculate(
                                pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity)))
                .withName("running Pivot"));
  }

  public Command manualPivot(Supplier<Double> joystick) {
    return run(
        () -> {
          double velocity = joystick.get() * PivotConstants.MAX_VELOCITY.in(Units.RadiansPerSecond);
          double periodMovement = Constants.PERIOD.in(Units.Second) * velocity;
          double draftSetpoint = periodMovement + pivot.getPosition().getRadians();
          double setpoint =
              Math.max(
                  Math.min(PivotConstants.MAX_ANGLE.in(Units.Radians), draftSetpoint),
                  PivotConstants.MIN_ANGLE.in(Units.Radians));
          pivot.setVoltage(
              pivotPID.calculate(pivot.getPosition().getRadians(), setpoint)
                  + pivotFeedforward.calculate(setpoint, 0));
        });
  }

  public Command climb(Supplier<Rotation2d> goalAngle) {
    return runOnce(() -> climbPID.setGoal(goalAngle.get().getRadians()))
        .andThen(
            run(() ->
                    pivot.setVoltage(
                        climbPID.calculate(pivot.getPosition().getRadians())
                            + climbFeedforward.calculate(
                                climbPID.getSetpoint().position, climbPID.getSetpoint().velocity)))
                .withName("running Climb"));
  }

  @Log.NT
  private double positionRadians() {
    return getPosition().getRadians();
  }

  public Rotation2d getPosition() {
    return pivot.getPosition();
  }

  public boolean atSetpoint() {
    return pivotPID.atSetpoint();
  }

  // ProfilePID doesn't log this stuff
  @Log.NT
  private double setpointRadians() {
    return pivotPID.getSetpoint().position;
  }

  @Override
  public void periodic() {
    positionVisualizer.setState(pivot.getPosition());
    setpointVisualizer.setState(Rotation2d.fromRadians(pivotPID.getSetpoint().position));
  }

  @Override
  public void close() throws Exception {
    pivot.close();
  }
}
