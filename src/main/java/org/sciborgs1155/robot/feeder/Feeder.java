package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

public class Feeder extends SubsystemBase implements AutoCloseable, Logged {
  private final FeederIO feeder;

  @Log.NT private final PIDController pid = new PIDController(kP, kI, kD);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

  private final SysIdRoutine sysId;

  public Feeder(FeederIO feeder) {
    this.feeder = feeder;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(v -> feeder.setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putData("feeder quasistatic backward", quasistaticBack());
    SmartDashboard.putData("feeder quasistatic forward", quasistaticForward());
    SmartDashboard.putData("feeder dynamic backward", dynamicBack());
    SmartDashboard.putData("feeder dynamic forward", dynamicForward());
  }

  public static Feeder create() {
    return Robot.isReal() ? new Feeder(new RealFeeder()) : new Feeder(new SimFeeder());
  }

  public Command runFeeder(double velocity) {
    return run(() -> {
          double ffOutput = ff.calculate(pid.getSetpoint(), velocity, Constants.PERIOD.in(Seconds));
          double pidOutput = pid.calculate(feeder.getVelocity(), velocity);
          feeder.setVoltage(pidOutput + ffOutput);
        })
        .withName("running feeder");
  }

  public Measure<Velocity<Angle>> getVelocity() {
    return RadiansPerSecond.of(feeder.getVelocity());
  }

  public Command quasistaticBack() {
    return sysId.quasistatic(Direction.kReverse);
  }

  public Command quasistaticForward() {
    return sysId.quasistatic(Direction.kForward);
  }

  public Command dynamicForward() {
    return sysId.dynamic(Direction.kForward);
  }

  public Command dynamicBack() {
    return sysId.dynamic(Direction.kReverse);
  }

  @Override
  public void close() throws Exception {
    feeder.close();
  }
}
