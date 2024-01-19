package org.sciborgs1155.robot.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.shooter.ShooterConstants.Flywheel;
import org.sciborgs1155.robot.shooter.ShooterConstants.Pivot;
import org.sciborgs1155.robot.shooter.feeder.FeederIO;
import org.sciborgs1155.robot.shooter.feeder.RealFeeder;
import org.sciborgs1155.robot.shooter.feeder.SimFeeder;
import org.sciborgs1155.robot.shooter.flywheel.FlywheelIO;
import org.sciborgs1155.robot.shooter.flywheel.RealFlywheel;
import org.sciborgs1155.robot.shooter.flywheel.SimFlywheel;
import org.sciborgs1155.robot.shooter.pivot.PivotIO;
import org.sciborgs1155.robot.shooter.pivot.RealPivot;
import org.sciborgs1155.robot.shooter.pivot.SimPivot;

public class Shooter extends SubsystemBase {
  private final FlywheelIO flywheel;
  private final FeederIO feeder;
  private final PivotIO pivot;

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

  public Command runFeeder(double voltage) {
    return run(() -> feeder.setVoltage(voltage));
  }

  public Command runFeederInverse(double voltage) {
    return runFeeder(voltage * -1);
  }

  // Make sure this is correct !!!
  public Command runFlywheel(DoubleSupplier velocity) {
    PIDController pid = new PIDController(Flywheel.kP, Flywheel.kI, Flywheel.kD);
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Flywheel.kS, Flywheel.kV, Flywheel.kA);
    return run(
        () ->
            flywheel.setVoltage(
                pid.calculate(flywheel.getVelocity(), velocity.getAsDouble())
                    + ff.calculate(velocity.getAsDouble())));
  }

  public Command runPivot(double goal) {
    ProfiledPIDController pid =
        new ProfiledPIDController(
            Pivot.kP,
            Pivot.kI,
            Pivot.kD,
            new TrapezoidProfile.Constraints(Pivot.MAX_VELOCITY, Pivot.MAX_ACCEL));
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Pivot.kS, Pivot.kV, Pivot.kA);
    return run(
        () ->
            pivot.setVoltage(
                pid.calculate(pivot.getPosition(), goal)
                    + ff.calculate(pid.getSetpoint().velocity)));
  }
}
