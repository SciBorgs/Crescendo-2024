package org.sciborgs1155.robot.pathfinding;

import static edu.wpi.first.units.Units.Centimeter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.awt.Polygon;
import java.awt.geom.*;
import java.util.List;
import java.util.ListIterator;

public class Obstacle {
  java.awt.Shape obstacle;

  Rectangle2D boundingBox = obstacle.getBounds2D();

  // The area in which the CENTER of the robot should not go, lest the EDGE of the robot will touch
  // the EDGE of the obstacle.
  public java.awt.Shape projectedObstacle =
      AffineTransform.getScaleInstance(
              // Finding which scale factor is needed in order to increase both opposite sides by
              // the robot radius (im a genius)
              (1 + PathfindingConstants.ROBOT_RADIUS.in(Centimeter) * 2 / boundingBox.getHeight()),
              (1 + PathfindingConstants.ROBOT_RADIUS.in(Centimeter) * 2 / boundingBox.getWidth()))
          .createTransformedShape(obstacle);

  // Scaling the obstacle to this factor. Increases all sides by the robot radius.

  /**
   * Obstacle constructor. If you want to make it easier for yourself, just use one of the other
   * static "constructor"
   *
   * @param obstacle An already-created Shape to turn into an Obstacle.
   */
  public Obstacle(java.awt.Shape obstacle) {
    this.obstacle = obstacle;
  }

  /**
   * Instantiates a new obstacle in the shape of a polygon.
   *
   * @param points A Translation2d list representing the positions in 2D space of which connect to
   *     make up the polygon in question.
   * @return An obstacle in the shape of a polygon. Its coordinates are all converted to integers,
   *     so make sure that a small distance measurement is used (centimeters or inches).
   */
  public static Obstacle polygonObstacle(List<Translation2d> points) {
    int[] xPoints = new int[points.size()];
    int[] yPoints = new int[points.size()];

    ListIterator<Translation2d> iterator = points.listIterator();

    while (iterator.hasNext()) {
      xPoints[iterator.nextIndex()] = (int) points.get(iterator.nextIndex()).getX();
      yPoints[iterator.nextIndex()] = (int) points.get(iterator.nextIndex()).getY();

      iterator.next();
    }

    return new Obstacle(new Polygon(xPoints, yPoints, 3));
  }

  /**
   * Instantiates a new Obstacle in the shape of a circle.
   *
   * @param position A Translation2d representing the position in 2D space of the CENTER of the
   *     ellipse.
   * @param height A double that represents the height of the ellipse in 2D space.
   * @param width A double that represents the width of the ellipse in 2D space.
   * @return An obstacle in the shape of a circle.
   */
  public static Obstacle ellipticalObstacle(Translation2d position, double height, double width) {
    return new Obstacle(
        new Ellipse2D.Double(
            // Finding the coordinates at the top left corner of the frame rectangle
            position.getX() - width / 2, position.getY() + height / 2, height, width));
  }

  /**
   * Instantiates a new Obstacle in the shape of a rotated ellipse.
   *
   * @param position A Pose2d representing the position in 2D space of the CENTER of the ellpise.
   * @param height A double that represents the height of the ellipse in 2D space.
   * @param width A double that represents the width of the ellipse in 2D space.
   * @return An obstacle in the shape of an ellipse, rotated about its center.
   */
  public static Obstacle ellipticalObstacle(Pose2d position, double height, double width) {
    return new Obstacle(
        AffineTransform.getRotateInstance(
                // Getting the rotation transformation
                position.getRotation().getRadians(), position.getX(), position.getY())
            .createTransformedShape(
                // Creating the ellipse
                new Ellipse2D.Double(
                    // Finding the coordinates at the top left corner of the frame rectangle
                    position.getX() - width / 2, position.getY() + height / 2, height, width)));
  }

  /**
   * Instantiates a new Obstacle in the shape of a rectangle.
   *
   * @param position A Translation2d representing the position in 2D space of the CENTER of the
   *     rectangle.
   * @param height A double that represents the height of the rectangle in 2D space.
   * @param width A double that represents the width of the rectangle in 2D space.
   * @return An obstacle in the shape of a rectangle, rotated about its center.
   */
  public static Obstacle rectangularObstacle(Translation2d position, double height, double width) {
    return new Obstacle(
        new Rectangle2D.Double(
            // Finding the coordinates at the top left corner of the rectangle
            position.getX() - width / 2, position.getY() + height / 2, height, width));
  }

  /**
   * Instantiates a new Obstacle in the shape of a rotated rectangle.
   *
   * @param position A Pose2d representing the position in 2D space of the CENTER of the rectangle.
   * @param height A double that represents the height of the rectangle in 2D space.
   * @param width A double that represents the width of the rectangle in 2D space.
   * @return An obstacle in the shape of a circle.
   */
  public static Obstacle rectangularObstacle(Pose2d position, double height, double width) {
    return new Obstacle(
        AffineTransform.getRotateInstance(
                // Getting the rotation transformation
                position.getRotation().getRadians(), position.getX(), position.getY())
            .createTransformedShape(
                // Creating the rectangle
                new Rectangle2D.Double(
                    // Finding the coordinates at the top left corner
                    position.getX() - width / 2, position.getY() + height / 2, height, width)));
  }

  /**
   * Instantiates a new Obstacle in the shape of a circle.
   *
   * @param position The position in 2D space of the center of the circle.
   * @param radius The radius in 2D space of the circle.
   * @return An obstacle in the shape of a circle.
   */
  public static Obstacle circularObstacle(Translation2d position, double radius) {
    return ellipticalObstacle(position, radius, radius);
  }

  /**
   * @return The area at which the center of the robot shall not go lest it collide with the
   *     obstacle.
   */
  public java.awt.Shape obstacleProjection() {
    return projectedObstacle;
  }
}
