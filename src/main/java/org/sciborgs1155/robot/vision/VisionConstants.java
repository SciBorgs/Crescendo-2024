package org.sciborgs1155.robot.vision;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.sciborgs1155.robot.vision.Vision.CameraConfig;

public class VisionConstants {
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  /* The front of the robot is defined at the intake */

  public static final CameraConfig BACK_LEFT_CAMERA =
      new CameraConfig(
          "back left",
          new Transform3d(
              new Translation3d(Inches.of(-10.74), Inches.of(11.617), Inches.of(17.25)),
              new Rotation3d(0, -Math.PI / 6, 5 * Math.PI / 6)));
  public static final CameraConfig BACK_RIGHT_CAMERA =
      new CameraConfig(
          "back right",
          new Transform3d(
              new Translation3d(Inches.of(-10.74), Inches.of(-11.617), Inches.of(17.25)),
              new Rotation3d(0, -Math.PI / 6, -5 * Math.PI / 6)));
  public static final CameraConfig FRONT_LEFT_CAMERA =
      new CameraConfig(
          "front left",
          new Transform3d(
              new Translation3d(-0.089, 0.3, 0.46),
              new Rotation3d(Math.PI, -Math.PI / 6, Math.PI / 6)));
  public static final CameraConfig FRONT_RIGHT_CAMERA =
      new CameraConfig(
          "front right",
          new Transform3d(
              new Translation3d(-0.273, -0.3, 0.485),
              new Rotation3d(0, -Math.PI / 6, -Math.PI / 6)));

  // OV9281 constants for our configuration
  public static final int WIDTH = 1280;
  public static final int HEIGHT = 800;
  public static final Rotation2d FOV = Rotation2d.fromDegrees(70);

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 7);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

  public static final double MAX_HEIGHT = 0.305;
  public static final double MAX_ANGLE = 0.3;

  // Total of 16 AprilTags
  // https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf (page 35 and more)
  // Tag Locations (1-16) | Source: 1,2,9,10 | Speaker: 3,4,7,8 | Amp: 5,6 | Stage 11,12,13,14,15,16
  // First half of locations are on red side, second half on blue side
  // (ex. source: 1,2 is red, 9,10)

  // Unsure if getFudicialID() returns the tags from 1-16 or 0-15 (assuming 0-15)
  public static final double[] TAG_WEIGHTS = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
}
