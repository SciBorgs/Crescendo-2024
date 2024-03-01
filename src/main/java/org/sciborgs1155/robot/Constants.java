package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
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

  /** Represents the model of robot this code is controlling. */
  public static enum RobotType {
    COMPLETE,
    CHASSIS,
    NO_PIVOT
  }

  /** The current model of robot this code is controlling. */
  public static final RobotType ROBOT_TYPE = RobotType.COMPLETE;

  /** Returns the robot's alliance. */
  public static Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  /** Returns the rotation of the robot's alliance with respect to the origin. */
  public static Rotation2d allianceRotation() {
    return Rotation2d.fromRotations(alliance() == Alliance.Blue ? 0 : 0.5);
  }

  public static final Measure<Time> PERIOD = Seconds.of(0.02); // roborio tickrate (s)
  public static final double DEADBAND = 0.15;
  public static final double MAX_RATE =
      DriveConstants.MAX_ACCEL.baseUnitMagnitude()
          / DriveConstants.MAX_ANGULAR_SPEED.baseUnitMagnitude();
  public static final double SLOW_SPEED_MULTIPLIER = 0.33;
  public static final double FULL_SPEED_MULTIPLIER = 1.0;

  // Origin at corner of blue alliance side of field
  public static class Field {
    public static final Measure<Distance> LENGTH = Inches.of(651.223);
    public static final Measure<Distance> WIDTH = Inches.of(323.277);

    public static final Translation3d BLUE_SPEAKER_POSE =
        new Translation3d(-0.086473, 5.757474, 2.1);
    public static final Translation3d RED_SPEAKER_POSE =
        new Translation3d(16.389722, 5.757474, 2.1);

    // found from
    // https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/FieldConstants.java
    public static Pose3d BLUE_LEFT_NOTE =
        new Pose3d(new Translation3d(2.9, 4.11, 0), new Rotation3d());
    public static Pose3d BLUE_MID_NOTE =
        new Pose3d(new Translation3d(2.9, 5.55, 0), new Rotation3d());
    public static Pose3d BLUE_RIGHT_NOTE =
        new Pose3d(new Translation3d(2.9, 7.0, 0), new Rotation3d());

    public static Pose3d RED_LEFT_NOTE =
        new Pose3d(new Translation3d(13.642, 4.11, 0), new Rotation3d());
    public static Pose3d RED_MID_NOTE =
        new Pose3d(new Translation3d(13.642, 5.55, 0), new Rotation3d());
    public static Pose3d RED_RIGHT_NOTE =
        new Pose3d(new Translation3d(13.642, 7.0, 0), new Rotation3d());

    // left to right, facing forward towards field from origin
    public static Pose3d CENTER_NOTE_ONE =
        new Pose3d(new Translation3d(8.2705321, 0.7528052, 0), new Rotation3d());
    public static Pose3d CENTER_NOTE_TWO =
        new Pose3d(new Translation3d(8.2705321, 2.4292052, 0), new Rotation3d());
    public static Pose3d CENTER_NOTE_THREE =
        new Pose3d(new Translation3d(8.2705321, 4.1056052, 0), new Rotation3d());
    public static Pose3d CENTER_NOTE_FOUR =
        new Pose3d(new Translation3d(8.2705321, 5.78201, 0), new Rotation3d());
    public static Pose3d CENTER_NOTE_FIVE =
        new Pose3d(new Translation3d(8.2705321, 7.4584052, 0), new Rotation3d());

    /** Returns the translation of the speaker for the robot's alliance. */
    public static Translation3d speaker() {
      return alliance() == Alliance.Red ? RED_SPEAKER_POSE : BLUE_SPEAKER_POSE;
    }

    public static boolean inField(Pose3d pose) {
      return (pose.getX() > 0
          && pose.getX() < Field.LENGTH.in(Meters)
          && pose.getY() > 0
          && pose.getY() < Field.WIDTH.in(Meters));
    }
  }
}
