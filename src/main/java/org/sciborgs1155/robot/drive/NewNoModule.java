package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NewNoModule implements ModuleIO {

    @Override
    public void setDriveVoltage(double voltage) {}

    @Override
    public void setTurnVoltage(double voltage) {}

    @Override
    public double drivePosition() {
        return 0;
    }

    @Override
    public double driveVelocity() {
        return 0;
    }

    @Override
    public Rotation2d rotation() {
        return new Rotation2d();
    }

    @Override
    public void resetEncoders() {}

    @Override
    public SwerveModuleState state() {
        return new SwerveModuleState();
    }

    @Override
    public SwerveModulePosition position() {
        return new SwerveModulePosition();
    }

    @Override
    public SwerveModuleState desiredState() {
        return new SwerveModuleState();
    }

    @Override
    public void setDriveSetpoint(double velocity) {}

    @Override
    public void setTurnSetpoint(double angle) {}

    @Override
    public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {}

    @Override
    public void close() {}
}
