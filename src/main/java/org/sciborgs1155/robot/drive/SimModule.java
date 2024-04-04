package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class SimModule implements ModuleIO {

  private final DCMotorSim drive =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Driving.FF.V, Driving.FF.kA_linear),
          DCMotor.getNeoVortex(1),
          1 / Driving.GEARING);
  private final DCMotorSim turn =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Turning.FF.V, Turning.FF.A),
          DCMotor.getNeo550(1),
          1 / Turning.MOTOR_GEARING);

  @Override
  public void setDriveVoltage(double voltage) {
    drive.setInputVoltage(voltage);
    drive.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turn.setInputVoltage(voltage);
    turn.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double drivePosition() {
    return drive.getAngularPositionRad();
  }

  @Override
  public double driveVelocity() {
    return drive.getAngularVelocityRadPerSec();
  }

  @Override
  public Rotation2d rotation() {
    return Rotation2d.fromRadians(turn.getAngularPositionRad());
  }

  @Override
  public void resetEncoders() {
    drive.setState(VecBuilder.fill(0, 0));
    turn.setState(VecBuilder.fill(0, 0));
  }

  @Override
  public void close() {}
}
