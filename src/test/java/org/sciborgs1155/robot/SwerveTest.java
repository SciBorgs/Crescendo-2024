package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.GyroIO;
import org.sciborgs1155.robot.drive.SimModule;

/** Swerve test. Currently incomplete and does nothing. */
public class SwerveTest {
  SimModule frontLeft;
  SimModule frontRight;
  SimModule rearLeft;
  SimModule rearRight;
  GyroIO.NoGyro gyro;
  Drive drive;

  final double DELTA = 5e-1;

  @BeforeEach
  public void setup() {
    setupHAL();
    frontLeft = new SimModule();
    frontRight = new SimModule();
    rearLeft = new SimModule();
    rearRight = new SimModule();
    gyro = new GyroIO.NoGyro();
    drive = new Drive(frontLeft, frontRight, rearLeft, rearRight);
    drive.resetEncoders();
  }

  @Disabled
  @Test
  public void reachesModuleSetpoint() {
    SwerveModuleState state = new SwerveModuleState(2, Rotation2d.fromRadians(Math.PI / 3));
    SwerveModuleState[] states = new SwerveModuleState[] {state, state, state, state};

    drive.setModuleStates(states);
    fastForward(1000);

    states = drive.getModuleStates();
    assertEquals(state.angle.getRadians(), states[1].angle.getRadians(), DELTA);
    assertEquals(state.speedMetersPerSecond, states[1].speedMetersPerSecond, DELTA);
  }

  @Test
  public void correctModulePosition() {
    double xVelocitySetpoint = 3;
    double yVelocitySetpoint = 0;
    run(drive.drive(() -> xVelocitySetpoint, () -> yVelocitySetpoint, drive::getHeading));

    fastForward(); // 200 ticks, 50 ticks/s = 4 seconds

    SwerveModulePosition[] positions = drive.getModulePositions();

    assertEquals(xVelocitySetpoint * 4, positions[1].distanceMeters, DELTA);
    // check for angle pos
  }
}
