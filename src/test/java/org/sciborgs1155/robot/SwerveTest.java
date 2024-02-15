package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
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

  final double DELTA = 0.15;

  @BeforeEach
  public void setup() {
    setupHAL();
    frontLeft = new SimModule();
    frontRight = new SimModule();
    rearLeft = new SimModule();
    rearRight = new SimModule();
    gyro = new GyroIO.NoGyro();
    drive = new Drive(gyro, frontLeft, frontRight, rearLeft, rearRight);
    drive.resetEncoders();
  }

  @Test
  public void reachesRobotVelocity() {
    double xVelocitySetpoint = -2;
    double yVelocitySetpoint = 4;
    run(drive.drive(() -> xVelocitySetpoint, () -> yVelocitySetpoint, drive::getHeading));
    fastForward(200);

    ChassisSpeeds chassisSpeed = drive.getChassisSpeed();

    assertEquals(xVelocitySetpoint, chassisSpeed.vxMetersPerSecond, DELTA);
    assertEquals(yVelocitySetpoint, chassisSpeed.vyMetersPerSecond, DELTA);
  }

  @ParameterizedTest
  @ValueSource(doubles = {3, 4})
  public void reachesAngularVelocity(double omegaRadiansPerSecond) {
    run(drive.drive(() -> 0, () -> 0, () -> omegaRadiansPerSecond));
    fastForward();

    ChassisSpeeds chassisSpeed = drive.getChassisSpeed();
    assertEquals(omegaRadiansPerSecond, chassisSpeed.omegaRadiansPerSecond, DELTA);
  }

  @Test
  public void testModuleDistance() {
    double xVelocitySetpoint = 2.265;
    double yVelocitySetpoint = 0;

    double deltaX = xVelocitySetpoint * 4;
    double deltaY = yVelocitySetpoint * 4;
    run(drive.drive(() -> xVelocitySetpoint, () -> yVelocitySetpoint, drive::getHeading));

    fastForward(); // 200 ticks, 50 ticks/s = 4 seconds

    SwerveModulePosition[] positions = drive.getModulePositions();
    Pose2d pose = drive.getPose();

    assertEquals(Math.hypot(deltaX, deltaY), positions[0].distanceMeters, DELTA);
    assertEquals(Math.hypot(deltaX, deltaY), positions[1].distanceMeters, DELTA);
    assertEquals(Math.hypot(deltaX, deltaY), positions[2].distanceMeters, DELTA);
    assertEquals(Math.hypot(deltaX, deltaY), positions[3].distanceMeters, DELTA);
    assertEquals(deltaX, pose.getX(), DELTA);
    assertEquals(deltaY, pose.getY(), DELTA);
  }
}
