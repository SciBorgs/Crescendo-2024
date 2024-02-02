package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.junit.jupiter.api.BeforeEach;
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

  final double DELTA = 1.5e-1;

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

  @Test
  public void reachesRobotVelocity() {
    double xVelocitySetpoint = 1.155;
    double yVelocitySetpoint = 2.265;
    run(drive.drive(() -> xVelocitySetpoint, () -> yVelocitySetpoint, drive::getHeading));
    fastForward(200);

    ChassisSpeeds chassisSpeed = drive.getChassisSpeed();
    assertEquals(
        Math.hypot(xVelocitySetpoint, yVelocitySetpoint),
        Math.hypot(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond),
        DELTA);
  }

  @Test
  public void reachesAngularVelocity() {
    double omegaRadiansPerSecond = 1.155;
    run(drive.drive(() -> 1, () -> 0, () -> omegaRadiansPerSecond));
    fastForward();

    ChassisSpeeds chassisSpeed = drive.getChassisSpeed();
    assertEquals(omegaRadiansPerSecond, chassisSpeed.omegaRadiansPerSecond, DELTA);
  }

  @Test
  public void testModuleDistance() {
    double xVelocitySetpoint = 2.265;
    double yVelocitySetpoint = 0;
    run(drive.drive(() -> xVelocitySetpoint, () -> yVelocitySetpoint, drive::getHeading));

    fastForward(); // 200 ticks, 50 ticks/s = 4 seconds

    SwerveModulePosition[] positions = drive.getModulePositions();

    assertEquals(
        Math.hypot(xVelocitySetpoint * 4, yVelocitySetpoint * 4),
        positions[0].distanceMeters,
        DELTA);
    assertEquals(
        Math.hypot(xVelocitySetpoint * 4, yVelocitySetpoint * 4),
        positions[1].distanceMeters,
        DELTA);
    assertEquals(
        Math.hypot(xVelocitySetpoint * 4, yVelocitySetpoint * 4),
        positions[2].distanceMeters,
        DELTA);
    assertEquals(
        Math.hypot(xVelocitySetpoint * 4, yVelocitySetpoint * 4),
        positions[3].distanceMeters,
        DELTA);
  }
}
