package org.sciborgs1155.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;
import monologue.Logged;
import org.photonvision.PhotonCamera;
import org.sciborgs1155.lib.FaultLogger;

public class NoteVision implements Logged {
  private final PhotonCamera camera;

  private Translation2d last = new Translation2d();

  public NoteVision() {
    camera = new PhotonCamera(VisionConstants.NOTE_CAMERA_NAME);

    FaultLogger.register(camera);
  }

  public void resetLast() {
    last = new Translation2d();
  }

  public Optional<Translation2d> nearestNote() {
    Optional<Translation2d> note =
        Optional.ofNullable(camera.getLatestResult().getBestTarget())
            .map(t -> t.getBestCameraToTarget().getTranslation().toTranslation2d());
    if (note.isPresent()) {
      last = note.get();
    }
    return note;
  }

  public Translation2d lastNote() {
    return last;
  }

  public Translation2d note() {
    return nearestNote().orElse(lastNote());
  }
}
