package org.sciborgs1155.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import org.sciborgs1155.robot.drive.Drive;

public class Path {
  public final Pose2d endingPoint;

  private static final AffineTransform nullTransform = new AffineTransform();

  private GriddedField field;

  private Drive drive;

  Supplier<Pose2d> pose;

  public Path(Pose2d startingPoint, Pose2d endingPoint, GriddedField field, Drive drive) {
    this.endingPoint = endingPoint;
    this.field = field;
    this.drive = drive;
  }

  /**
   * Updates the moving obstacles.
   *
   * @param obstacles The new obstacles that represent the moving ones.
   */
  public void updateObstacles(Supplier<List<Obstacle>> obstacles) {
    field.addTempObstacles(obstacles.get());
  }

  /**
   * Checks whether the robot is currently at the endpoint.
   *
   * @return True if the robot is at the endpoint, false if it isn't.
   */
  public boolean atEndPoint() {
    return drive.pose().getTranslation().getDistance(endingPoint.getTranslation())
        < PathfindingConstants.GRID_SIDE_LENGTH;
  }

  /**
   * Static method to find out whether a point can be driven straight to from another point.
   *
   * @param startPoint The starting point.
   * @param setPoint The ending point.
   * @return whether the ending point can be driven straight to from the starting point.
   */
  public static boolean pointSeeable(
      Pose2d startPoint, Pose2d setPoint, List<Obstacle> obstaclelist) {
    AtomicBoolean seeable = new AtomicBoolean(true);
    Line2D beeline =
        new Line2D.Double(startPoint.getX(), startPoint.getY(), setPoint.getX(), setPoint.getY());
    obstaclelist.forEach(
        obstacle ->
            seeable.set(
                seeable.get() && shapeIntersectsLine(obstacle.obstacleProjection(), beeline)));
    return seeable.get();
  }

  /**
   * Static method to figure out whether a shape is intersected by a line.
   *
   * @param shape The shape in question.
   * @param line The line, previously mentioned.
   * @return True if the shape intersects the line, and false if it does not.
   */
  public static boolean shapeIntersectsLine(java.awt.Shape shape, Line2D line) {
    PathIterator iterator = shape.getPathIterator(nullTransform);
    double[] currCoords = new double[6];
    double[] prevCoords = new double[2];

    iterator.currentSegment(prevCoords);

    while (!iterator.isDone()) {
      int segmentType = iterator.currentSegment(currCoords);

      if (segmentType != PathIterator.SEG_MOVETO) {
        Line2D.Double segment =
            new Line2D.Double(prevCoords[0], prevCoords[1], currCoords[0], currCoords[1]);

        if (segment.intersectsLine(line)) {
          return true;
        }
      }

      prevCoords[0] = currCoords[0];
      prevCoords[0] = currCoords[1];

      iterator.next();
    }
    return false;
  }

  /**
   * Nonstatic method to find out whether the goal of the robot can be driven straight to from where
   * the robot is.
   *
   * @return whether the goal can be driven straight to.
   */
  public boolean setPointSeeable(Pose2d startingPoint) {
    return pointSeeable(startingPoint, endingPoint, PathfindingConstants.stationaryObstacles);
  }
}
