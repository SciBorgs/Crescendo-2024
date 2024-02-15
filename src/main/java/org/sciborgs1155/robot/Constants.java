package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.sciborgs1155.robot.drive.DriveConstants;

/**
 * Constants is a globally accessible class for storing immutable values. Every value should be
 * <code>public static final</code>.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * @see Units
 */
public class Constants {
  // Origin at corner of blue alliance side of field
  public static class Field {
    public static final Translation3d BLUE_SPEAKER_POSE =
        new Translation3d(-0.086473, 5.757474, 2.1);
    public static final Translation3d RED_SPEAKER_POSE =
        new Translation3d(16.389722, 5.757474, 2.1);

    // found from
    // https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/FieldConstants.java
    public static Translation2d BLUE_LEFT_NOTE = new Translation2d(2.9, 4.11);
    public static Translation2d BLUE_MID_NOTE = new Translation2d(2.9, 5.55);
    public static Translation2d BLUE_RIGHT_NOTE = new Translation2d(2.9, 7.0);

    // left to right, facing forward towards field from origin
    public static Translation2d CENTER_NOTE_ONE = new Translation2d(8.2705321, 0.7528052);
    public static Translation2d CENTER_NOTE_TWO = new Translation2d(8.2705321, 2.4292052);
    public static Translation2d CENTER_NOTE_THREE = new Translation2d(8.2705321, 4.1056052);
    public static Translation2d CENTER_NOTE_FOUR = new Translation2d(8.2705321, 5.78201);
    public static Translation2d CENTER_NOTE_FIVE = new Translation2d(8.2705321, 7.4584052);

    public static Translation3d getSpeaker() {
      if (DriverStation.getAlliance().isPresent()) {
        return DriverStation.getAlliance().get() == Alliance.Red
            ? RED_SPEAKER_POSE
            : BLUE_SPEAKER_POSE;
      } else {
        return BLUE_SPEAKER_POSE;
      }
    }
  }

  public static enum RobotType {
    COMPLETE,
    CHASSIS,
    NO_PIVOT
  }

  public static final RobotType ROBOT_TYPE = RobotType.COMPLETE;

  public static final Measure<Time> PERIOD = Seconds.of(0.02); // roborio tickrate (s)
  public static final double DEADBAND = 0.1;
  public static final double MAX_RATE =
      DriveConstants.MAX_ACCEL.baseUnitMagnitude()
          / DriveConstants.MAX_ANGULAR_SPEED.baseUnitMagnitude();
  public static final double SLOW_SPEED_MULTIPLIER = 0.33;
  public static final double FULL_SPEED_MULTIPLIER = 1.0;
}
