package org.sciborgs1155.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.sciborgs1155.robot.Robot;

public class Vision {
  public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}

  private List<PhotonCamera> cameras = new ArrayList<PhotonCamera>();
  private List<PhotonPoseEstimator> poseEstimators = new ArrayList<PhotonPoseEstimator>();
  private List<PhotonCameraSim> simCameras = new ArrayList<PhotonCameraSim>();

  private Set<Double> timeStamps = new HashSet<>();
  private VisionSystemSim visionSim;

  public Vision(VisionConstants.CameraConfig... cameras) {
    // this.frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
    // this.rearCamera = new PhotonCamera(VisionConstants.REAR_CAMERA_NAME);
    boolean isSimulation = Robot.isSimulation();
    if (isSimulation) {
      this.visionSim = new VisionSystemSim("main");
      this.visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);
    }

    for (int i = 0; i < cameras.length; i++) {
      PhotonCamera curCamera = new PhotonCamera(cameras[i].name());
      PhotonPoseEstimator curPoseEstimator =
          new PhotonPoseEstimator(
              VisionConstants.TAG_LAYOUT,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              curCamera,
              cameras[i].robotToCam());

      curPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      // Adding cameras and pose estimators to respective lists
      this.cameras.add(curCamera);
      this.poseEstimators.add(curPoseEstimator);

      if (isSimulation) {
        // If robot is in simulation, cameraSims will be instantiated and added to respective list
        PhotonCameraSim cameraSim;
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);

        cameraSim = new PhotonCameraSim(curCamera, cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        this.visionSim.addCamera(cameraSim, cameras[i].robotToCam());
        this.simCameras.add(cameraSim);
        cameraSim.enableDrawWireframe(true);
      }
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public EstimatedRobotPose[] getEstimatedGlobalPoses() {
    EstimatedRobotPose[] poseEstimations =
        poseEstimators.stream()
            .map(PhotonPoseEstimator::update)
            .flatMap(Optional::stream)
            .filter(
                (curPose) ->
                    timeStamps.contains(
                        curPose.timestampSeconds)) // Filters out all duplicate positions
            .toArray(EstimatedRobotPose[]::new);

    for (EstimatedRobotPose curPose : poseEstimations) {
      // Once all duplicate positions are been removed, current timestamp is cached
      timeStamps.add(curPose.timestampSeconds);
    }
    ;
    return poseEstimations;
  }

  public PoseEstimate[] getPoseEstimates(Pose2d estimatedDrivePose) {
    EstimatedRobotPose[] curEstimatedPoses = getEstimatedGlobalPoses();
    int length = cameras.size();
    PoseEstimate[] poseEstimates = new PoseEstimate[length];
    for (int i = 0; i < length; i++) {
      // Fetch every indivdual camera along with its respective pose estimator
      PhotonCamera curCamera = this.cameras.get(i);
      PhotonPoseEstimator curEstimator = this.poseEstimators.get(i);

      Matrix<N3, N1> estimatedStdDev =
          getEstimationStdDevs(estimatedDrivePose, curCamera, curEstimator);
      poseEstimates[i] = new PoseEstimate(curEstimatedPoses[i], estimatedStdDev);
    }
    return poseEstimates;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, PhotonCamera camera, PhotonPoseEstimator estimator) {
    var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    var targets = camera.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visibleX
    if (numTags > 1) estStdDevs = VisionConstants.MULTIPLE_TAG_STD_DEVS;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }
}
