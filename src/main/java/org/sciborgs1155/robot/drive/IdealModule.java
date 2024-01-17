package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.List;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Constants;

/** Ideal swerve module, useful for debugging */
public class IdealModule implements ModuleIO {

  private SwerveModuleState state = new SwerveModuleState();
  private double distance;

  @Override
  public SwerveModuleState getState() {
    return state;
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(distance, state.angle);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    state = SwerveModuleState.optimize(desiredState, state.angle);
    distance += state.speedMetersPerSecond * Constants.PERIOD;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return state;
  }

  @Override
  public void resetEncoders() {}

  @Override
  public void setTurnPID(double kP, double kI, double kD) {}

  @Override
  public void setDrivePID(double kP, double kI, double kD) {}

  @Override
  public void close() {}

  @Override
  public List<HardwareFault> getFaults() {
    return List.of();
  }
}
