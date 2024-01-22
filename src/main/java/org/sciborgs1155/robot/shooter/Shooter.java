package org.sciborgs1155.robot.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.shooter.ShooterConstants.FlywheelConstants;
import org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants;
import org.sciborgs1155.robot.shooter.ShooterConstants.PivotConstants.ClimbConstants;
import org.sciborgs1155.robot.shooter.feeder.FeederIO;
import org.sciborgs1155.robot.shooter.feeder.RealFeeder;
import org.sciborgs1155.robot.shooter.feeder.SimFeeder;
import org.sciborgs1155.robot.shooter.flywheel.FlywheelIO;
import org.sciborgs1155.robot.shooter.flywheel.RealFlywheel;
import org.sciborgs1155.robot.shooter.flywheel.SimFlywheel;
import org.sciborgs1155.robot.shooter.pivot.PivotIO;
import org.sciborgs1155.robot.shooter.pivot.RealPivot;
import org.sciborgs1155.robot.shooter.pivot.SimPivot;
import org.sciborgs1155.robot.shooter.pivot.Visualizer;

public class Shooter extends SubsystemBase {
  private final FlywheelIO flywheel;
  private final FeederIO feeder;
  private final PivotIO pivot;

  final Mechanism2d mech = new Mechanism2d(3, 4);
  private final Visualizer positionVisualizer = new Visualizer(null, null);
  private final Visualizer setpointVisualizer = new Visualizer(null, null);

  private final PIDController flywheelPID = new PIDController(Flywheel.kP, Flywheel.kI, Flywheel.kD);
  private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(Flywheel.kS, Flywheel.kV, Flywheel.kA);

  public static Shooter create() {
    return Robot.isReal()
        ? new Shooter(new RealFlywheel(), new RealPivot(), new RealFeeder())
        : new Shooter(new SimFlywheel(), new SimPivot(), new SimFeeder());
  }

  public Shooter(FlywheelIO flywheel, PivotIO pivot, FeederIO feeder) {
    this.flywheel = flywheel;
    this.feeder = feeder;
    this.pivot = pivot;
  }

  public Command runFeeder(double speed) {
    return run(() -> feeder.set(speed)).withName("running Feeder");
  }

  public Command runFeederInverse(double voltage) {
    return runFeeder(voltage * -1).withName("running Feeder backwards");
  }

  // Make sure this is correct !!!
  public Command runFlywheel(DoubleSupplier velocity) {
<<<<<<< HEAD
    return run(() ->
            flywheel.setVoltage(
                flywheelPID.calculate(flywheel.getVelocity(), velocity.getAsDouble())
                    + flywheelFeedforward.calculate(velocity.getAsDouble())))
        .withName("running Flywheel");
=======
    PIDController pid =
        new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
    SimpleMotorFeedforward ff =
        new SimpleMotorFeedforward(
            FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);
    return run(() ->
            flywheel.setVoltage(
                pid.calculate(flywheel.getVelocity(), velocity.getAsDouble())
                    + ff.calculate(velocity.getAsDouble())))
        .finallyDo(() -> pid.close())
        .withName("running FlywheelConstants");
>>>>>>> f1d490810f8b198aee989a679e7ca3014fcd93e9
  }

  public Command runPivot(double goalAngle) {
    ProfiledPIDController pid =
        new ProfiledPIDController(
            PivotConstants.kP,
            PivotConstants.kI,
            PivotConstants.kD,
            new TrapezoidProfile.Constraints(
                PivotConstants.MAX_VELOCITY, PivotConstants.MAX_ACCEL));
    ArmFeedforward ff = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);

    return run(() ->
            pivot.setVoltage(
                pid.calculate(pivot.getPosition(), goalAngle)
                    + ff.calculate(
                        goalAngle + PivotConstants.POSITION_OFFSET, pid.getSetpoint().velocity)))
        .withName("running Pivot");
  }

  public Command climb(double goalAngle) {
    ProfiledPIDController pid =
        new ProfiledPIDController(
            ClimbConstants.kP,
            ClimbConstants.kI,
            ClimbConstants.kD,
            new TrapezoidProfile.Constraints(
                ClimbConstants.MAX_VELOCITY, ClimbConstants.MAX_ACCEL));
    ArmFeedforward ff = new ArmFeedforward(ClimbConstants.kS, ClimbConstants.kG, ClimbConstants.kV);

    return run(() ->
            pivot.setVoltage(
                pid.calculate(pivot.getPosition(), goalAngle)
                    + ff.calculate(goalAngle, pid.getSetpoint().velocity)))
        .withName("climbing. . .");
  }

  public double getFlywheelVelocity() {
    return flywheel.getVelocity();
  }

  public double getFeederVelocity() {
    return feeder.getVelocity();
  }

  public double getPivotPosition() {
    return pivot.getPosition();
  }

  @Override
  public void periodic() {
    positionVisualizer.setState(pivot.getPosition());
    setpointVisualizer.setState()
  }
}
