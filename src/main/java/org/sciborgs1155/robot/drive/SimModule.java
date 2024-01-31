package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class SimModule implements ModuleIO {

  private final DCMotorSim drive =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Driving.FF.V, Driving.FF.A),
          DCMotor.getNeo550(1),
          Driving.GEARING);
  private final DCMotorSim turn =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Turning.FF.V, Turning.FF.A),
          DCMotor.getNeo550(1),
          Turning.MOTOR_GEARING);

  private final MutableMeasure<Distance> drivePos = MutableMeasure.zero(Meters);
  private final MutableMeasure<Velocity<Distance>> driveVelocity =
      MutableMeasure.zero(MetersPerSecond);

  @Override
  public void setDriveVoltage(Measure<Voltage> voltage) {
    drive.setInputVoltage(voltage.in(Volts));
    drive.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public void setTurnVoltage(Measure<Voltage> voltage) {
    turn.setInputVoltage(voltage.in(Volts));
    turn.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public Measure<Distance> getDrivePosition() {
    return drivePos.mut_replace(drive.getAngularPositionRad(), Meters);
  }

  @Override
  public Measure<Velocity<Distance>> getDriveVelocity() {
    return driveVelocity.mut_replace(drive.getAngularVelocityRadPerSec(), MetersPerSecond);
  }

  @Override
  public Rotation2d getRotation() {
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
