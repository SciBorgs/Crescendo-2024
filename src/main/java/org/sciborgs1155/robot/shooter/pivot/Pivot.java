package org.sciborgs1155.robot.shooter.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
  private final ProfiledPIDController pivotPID;
  private final ArmFeedforward pivotFeedforward;

  @Log.NT
  private final PIDController manualPID =
      new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);

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

  public Command easyManualPivot(Supplier<Double> joystick) {
    return run(
        () -> {
          double periodMovement =
              Constants.PERIOD.in(Units.Second)
                  * joystick.get()
                  * PivotConstants.MAX_VELOCITY.in(Units.RadiansPerSecond);
          double draftSetpoint = periodMovement + pivot.getPosition().getRadians();
          double setpoint =
              Math.max(
                  Math.min(PivotConstants.MAX_ANGLE.in(Units.Radians), draftSetpoint),
                  PivotConstants.MIN_ANGLE.in(Units.Radians));
          pivot.setVoltage(manualPID.calculate(pivot.getPosition().getRadians(), setpoint));
        });
  }

  public Command manualPivot(Supplier<Double> joystick, Rotation2d maxRotation) {
    return run(
        () -> {
          double initVelocity = pivot.getVelocity();
          // gooberValue is the next max speed in order for the pivot to not go over the max angle.
          double gooberValue =
              initVelocity
                  - (Constants.PERIOD.in(Units.Second)
                      * 5
                      * PivotConstants.MAX_ACCEL.in(Units.RadiansPerSecond.per(Units.Second)));
          double initTheta = pivot.getPosition().getRadians();
          double deltaTheta =
              (Math.pow(initVelocity, 2)
                  / (2 * PivotConstants.MAX_ACCEL.in(Units.RadiansPerSecond.per(Units.Second))));

          boolean slow = maxRotation.getRadians() <= initTheta + deltaTheta;
          if (slow) {
            System.out.println("slowing!");
          } else {
            System.out.println("fastttt");
          }
          double velocity = slow ? Math.min(joystick.get(), gooberValue) : joystick.get();
          // double currentError = .getRadians() - (initTheta + deltaTheta));
          // double maxError = 0.1;
          pivot.setVoltage(manualPID.calculate(pivot.getVelocity(), velocity));
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

  public double getVelocity() {
    return pivot.getVelocity();
  }

  @Override
  public void periodic() {
    positionVisualizer.setState(pivot.getPosition().getDegrees());
    setpointVisualizer.setState(Degrees.convertFrom(pivotPID.getSetpoint().position, Radians));
  }

  @Override
  public void close() throws Exception {
    pivot.close();
  }
}
