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

  public static final CameraConfig LEFT_CAMERA =
      new CameraConfig(
          "left",
          new Transform3d(
              new Translation3d(Inches.of(-10.74), Inches.of(-11.617), Inches.of(18.147)),
              new Rotation3d(0, -Math.PI / 6, 5 * Math.PI / 6)));
  public static final CameraConfig RIGHT_CAMERA =
      new CameraConfig(
          "right",
          new Transform3d(
              new Translation3d(Inches.of(-10.74), Inches.of(11.617), Inches.of(18.147)),
              new Rotation3d(0, -Math.PI / 6, -5 * Math.PI / 6)));

  // OV9281 constants for our configuration
  public static final int WIDTH = 800;
  public static final int HEIGHT = 600;
  public static final Rotation2d FOV = Rotation2d.fromDegrees(100);

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 8);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.2, 0.2, 3);

  public static final double MAX_HEIGHT = 0.305;
  public static final double MIN_HEIGHT = 0.295;
}
