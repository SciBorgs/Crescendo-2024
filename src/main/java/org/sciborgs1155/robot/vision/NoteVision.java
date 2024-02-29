package org.sciborgs1155.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;
import monologue.Logged;
import org.photonvision.PhotonCamera;
import org.sciborgs1155.lib.FaultLogger;

public class NoteVision implements Logged {
  private final PhotonCamera camera;

  public NoteVision() {
    camera = new PhotonCamera(VisionConstants.NOTE_CAMERA_NAME);

    FaultLogger.register(camera);
  }

  public Optional<Translation2d> getNearestNote() {
    return Optional.ofNullable(camera.getLatestResult().getBestTarget())
        .map(t -> t.getBestCameraToTarget().getTranslation().toTranslation2d());
  }
}
