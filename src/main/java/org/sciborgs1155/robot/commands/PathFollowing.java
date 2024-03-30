package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Constants.Field.chainCoordinates;
import static org.sciborgs1155.robot.Constants.Field.speakerPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.function.Supplier;
import org.sciborgs1155.robot.Constants.Field;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.pathfinding.CharliesAstar;
import org.sciborgs1155.robot.pathfinding.GriddedField;
import org.sciborgs1155.robot.pathfinding.Obstacle;
import org.sciborgs1155.robot.pathfinding.Path;

public class PathFollowing {

  // How many periods before the robot updates the field and calculates a new path.
  // Increasing this number improves performance but also lengthens reaction time.
  private static final int REFRESH_INTERVAL = 5;

  // How close the robot will have to be before it shoots at the speaker.
  private static final Measure<Distance> SHOOTING_RANGE = Inches.of(100);

  private int refreshMeter = 5;

  private Alignment alignment;
  private Shooting shooting;
  private Path path;
  private Drive drive;
  private GriddedField field;
  private CharliesAstar aStar;

  public PathFollowing(Drive drive, GriddedField field, Alignment alignment, Shooting shooting) {
    this.drive = drive;
    this.field = field;
    this.alignment = alignment;
    this.shooting = shooting;
    aStar =
        new CharliesAstar(
            field,
            field.coordsToBox(new Translation2d(path.endingPoint.getX(), path.endingPoint.getY())));

    newPath(drive.pose());
  }

  /**
   * Sets the current path to a new one with a different goal.
   *
   * @param goal The new goal of the path.
   */
  public void newPath(Pose2d goal) {
    path = new Path(drive.pose(), goal, field, drive);
    refreshMeter = REFRESH_INTERVAL;
  }

  /**
   * Sends the robot to drive to the next point on its path toward the goal. Also finds the next
   * pose.
   *
   * @param speed The speed wanted at the next pose.
   * @return A command to send the robot to the next position in its heroic journey.
   */
  public Command goToNextPose() {
    if (refreshMeter >= REFRESH_INTERVAL) {
      aStar.assignCosts(field.coordsToBox(path.endingPoint.getTranslation()));
      refreshMeter = 0;
    } else {
      refreshMeter++;
    }
    return drive.driveIncrements(
        new Pose2d(aStar.nextPos(drive.pose().getTranslation()), path.endingPoint.getRotation()),
        path.endingPoint);
  }

  /**
   * Calculates a path from the starting point to the ending point which avoids all obstacles.
   *
   * @return A list of Translation2ds which represent the coordinate path of the robot.
   */
  public List<Translation2d> calculatePath() {
    field.addTempObstacles(field.getMovingObstacles());
    return aStar.pathMaker(drive.pose().getTranslation(), path.endingPoint.getTranslation());
  }

  /**
   * Updates the temporary onfield obstacles, then drives the robot to the next pose. The "inc"
   * means "incremental" but I shortened it to make the name shorter.
   *
   * @param movingObstacles The new temporary onfield obstacles.
   * @return A command to update obstacles then go to the next position in the path.
   */
  public Command incPathFollow(Supplier<List<Obstacle>> movingObstacles) {
    path.updateObstacles(movingObstacles);
    return goToNextPose();
  }

  /**
   * Follows the current path in the PathFollowing object.
   *
   * @param movingObstacles The new temporary onfield obstacles.
   * @return A command to repeatedly update obstacles, and then full follow the path.
   */
  public Command followPath(Supplier<List<Obstacle>> movingObstacles) {
    return incPathFollow(movingObstacles).until(path::atEndPoint);
  }

  public Command followNewPath(Supplier<List<Obstacle>> movingObstacles, Pose2d newGoal) {
    newPath(newGoal);
    return followPath(movingObstacles);
  }

  public Command goToSource(Supplier<List<Obstacle>> movingObstacles) {
    return followNewPath(movingObstacles, Field.sourceCoordinates());
  }

  public Command fullAutoAmp(Supplier<List<Obstacle>> movingObstacles) {
    return followNewPath(
            movingObstacles,
            new Pose2d(
                Field.ampCoordinates()
                    .plus(
                        new Translation2d(Inches.of(0), DriveConstants.CHASSIS_WIDTH.times(-0.5))),
                Rotation2d.fromRadians(-Math.PI / 2)))
        .andThen(alignment.ampAlign());
  }

  public Command fullAutoSpeaker(Supplier<List<Obstacle>> movingObstacles) {
    newPath(speakerPose());
    return incPathFollow(movingObstacles)
        .until(
            () ->
                Field.speaker().toTranslation2d().getDistance(drive.pose().getTranslation())
                        < SHOOTING_RANGE.in(Meters)
                    && path.setPointSeeable())
        .andThen(shooting.shootWhileDriving(() -> 0, () -> 0));
  }

  public Command fullAutoClimb(Supplier<List<Obstacle>> movingObstacles) {
    return followNewPath(movingObstacles, drive.pose().nearest(chainCoordinates()));
    // TODO right now it just goes to the nearest chain but doesn't climb. Will do that later.
  }

  public Command fullAutoSource(Supplier<List<Obstacle>> movingObstacles) {
    return followNewPath(movingObstacles, Field.sourceCoordinates());
  }
}
