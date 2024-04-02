package org.sciborgs1155.robot.vision;

import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.robot.Robot;

public class Vision implements Logged {
  public static record CameraConfig(String name, Transform3d robotToCam) {}

  public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}

  private final PhotonCamera[] cameras;
  private final PhotonPoseEstimator[] estimators;
  private final PhotonCameraSim[] simCameras;

  private VisionSystemSim visionSim;

  /** A factory to create new vision classes with our two configured cameras */
  public static Vision create() {
    return new Vision(BACK_LEFT_CAMERA, BACK_RIGHT_CAMERA);
  }

  public Vision(CameraConfig... configs) {
    cameras = new PhotonCamera[configs.length];
    estimators = new PhotonPoseEstimator[configs.length];
    simCameras = new PhotonCameraSim[configs.length];

    for (int i = 0; i < configs.length; i++) {
      PhotonCamera camera = new PhotonCamera(configs[i].name());
      PhotonPoseEstimator estimator =
          new PhotonPoseEstimator(
              VisionConstants.TAG_LAYOUT,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              camera,
              configs[i].robotToCam());

      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      cameras[i] = camera;
      estimators[i] = estimator;

      FaultLogger.register(camera);
    }

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);

      for (int i = 0; i < cameras.length; i++) {
        var prop = new SimCameraProperties();
        prop.setCalibration(WIDTH, HEIGHT, FOV);
        prop.setCalibError(0.15, 0.05);
        prop.setFPS(45);
        prop.setAvgLatencyMs(12);
        prop.setLatencyStdDevMs(3.5);

        PhotonCameraSim cameraSim = new PhotonCameraSim(cameras[i], prop);
        cameraSim.setMaxSightRange(5);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, configs[i].robotToCam());
        simCameras[i] = cameraSim;
      }
    }
  }

  /**
   * Returns a list of all currently visible pose estimates and their standard deviation vectors.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public PoseEstimate[] getEstimatedGlobalPoses() {
    List<PoseEstimate> estimates = new ArrayList<>();
    for (int i = 0; i < estimators.length; i++) {
      var result = cameras[i].getLatestResult();
      var estimate = estimators[i].update(result);
      log("estimates present " + i, estimate.isPresent());
      estimate
          .filter(
              f ->
                  Field.inField(f.estimatedPose)
                      && Math.abs(f.estimatedPose.getZ()) < MAX_HEIGHT
                      && Math.abs(f.estimatedPose.getRotation().getX()) < MAX_ANGLE
                      && Math.abs(f.estimatedPose.getRotation().getY()) < MAX_ANGLE)
          .ifPresent(
              e ->
                  estimates.add(
                      new PoseEstimate(
                          e, getEstimationStdDevs(e.estimatedPose.toPose2d(), result))));
    }
    return estimates.toArray(PoseEstimate[]::new);
  }

  /**
   * Returns the poses of all currently visible tags.
   *
   * @return An array of Pose3ds.
   */
  @Log.NT
  public Pose3d[] getSeenTags() {
    return Arrays.stream(cameras)
        .flatMap(c -> c.getLatestResult().targets.stream())
        .map(PhotonTrackedTarget::getFiducialId)
        .map(TAG_LAYOUT::getTagPose)
        .map(Optional::get)
        .toArray(Pose3d[]::new);
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, PhotonPipelineResult pipelineResult) {
    var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    var targets = pipelineResult.getTargets();
    int numTags = 0;
    double avgDist = 0;
    double avgWeight = 0;
    for (var tgt : targets) {
      var tagPose = TAG_LAYOUT.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      avgWeight += TAG_WEIGHTS[tgt.getFiducialId() - 1];
    }
    if (numTags == 0) return estStdDevs;

    avgDist /= numTags;
    avgWeight /= numTags;

    // Decrease std devs if multiple targets are visibleX
    if (numTags > 1) estStdDevs = VisionConstants.MULTIPLE_TAG_STD_DEVS;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    estStdDevs = estStdDevs.times(avgWeight);

    return estStdDevs;
  }

  /**
   * Updates the vision field simulation. This method should not be called when code is running on
   * the robot.
   */
  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }
}
