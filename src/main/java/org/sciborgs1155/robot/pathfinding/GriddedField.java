package org.sciborgs1155.robot.pathfinding;

import static edu.wpi.first.units.Units.Centimeter;
import static org.sciborgs1155.robot.pathfinding.PathfindingConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import org.sciborgs1155.robot.Constants.Field;
import java.awt.Shape;
import java.awt.geom.Rectangle2D;
import java.util.List;
import java.util.ListIterator;

public class GriddedField {
  // for ease of access and to make the coe look better

  public static final int INT_FIELD_LENGTH = (int) Field.LENGTH.in(Centimeter);
  public static final int INT_FIELD_WIDTH = (int) Field.WIDTH.in(Centimeter);

  public static final int LENGTH_GRID_NUMBER = (int) INT_FIELD_LENGTH / GRID_SIDE_LENGTH;
  public static final int WIDTH_GRID_NUMBER = (int) INT_FIELD_LENGTH / GRID_SIDE_LENGTH;

  public List<Obstacle> movingObstacles;

  /* Increasing the grid size to make it GRID_SIDE_LENGTH on each side, because otherwise
   * it would take up too much processing power and we don't need that level of precision. */
  private final GridBox[][] baseField = new GridBox[LENGTH_GRID_NUMBER][WIDTH_GRID_NUMBER];

  private GridBox[][] fullField = baseField;

  /**
   * Constructor. Instantiates a Gridded Field.
   *
   * @param stationaryObstacles A list of obstacles which contains all the permanent, stationary
   *     2-dimensional obstacles on the field.
   */
  public GriddedField() {
    movingObstacles = List.of();

    for (int x = 0; x < LENGTH_GRID_NUMBER; x++) {
      for (int y = 0; y < WIDTH_GRID_NUMBER; y++) {
        baseField[x][y] = new GridBox(x, y);
      }
    }

    // Turning the walls into obstacles.

    for (int x = 0; x < LENGTH_GRID_NUMBER; x++) {
      for (int y = 0; y < ROBOT_RADIUS; y++) {
        baseField[x / GRID_SIDE_LENGTH][y / GRID_SIDE_LENGTH].obstaclize();
        baseField[x / GRID_SIDE_LENGTH][LENGTH_GRID_NUMBER - y / GRID_SIDE_LENGTH].obstaclize();
      }
    }
    for (int x = 0; x < WIDTH_GRID_NUMBER; x++) {
      for (int y = 0; y < ROBOT_RADIUS; y++) {
        baseField[x / GRID_SIDE_LENGTH][y / GRID_SIDE_LENGTH].obstaclize();
        baseField[WIDTH_GRID_NUMBER - x / GRID_SIDE_LENGTH][y / GRID_SIDE_LENGTH].obstaclize();
      }
    }
  }

  /**
   * Adds temporary obstacles to the field. This particular method removes all temporary obstacles
   * currently on the field, then adds the ones inputted onto the field.
   *
   * @param obstacles A list of Obstacles, representing the temporary obstacles being placed on the
   *     field. These temporary obstacles are best for moving objects, where their position can be
   *     updated by clearing the field and re-placing them in their new position every period.
   */
  public void addTempObstacles(List<Obstacle> obstacles) {
    movingObstacles = obstacles;
    resetTemps();
    addObstacles(obstacles, fullField);
  }

  /**
   * A static method which adds temporary obstacles to a field. This method does not remove the
   * current temporary obstacles on the field.
   *
   * @param obstacles A list of Obstacles, representing the temporary obstacles being placed on the
   *     field. These temporary obstacles are best for moving objects, where their position can be
   *     updated by clearing the field and re-placing them in their new position every period.
   * @param field A GridBox two-dimensional array which represents the field to which the obstacles
   *     will be added into.
   */
  public static void addObstacles(List<Obstacle> obstacles, GridBox[][] field) {
    ListIterator<Obstacle> iterator = obstacles.listIterator();

    while (iterator.hasNext()) {
      Shape obstacle = obstacles.get(iterator.nextIndex()).projectedObstacle;
      Rectangle2D obstacleBoundingBox =
          obstacles.get(iterator.nextIndex()).projectedObstacle.getBounds2D();
      int boundingX = (int) obstacleBoundingBox.getX();
      int boundingY = (int) obstacleBoundingBox.getY();
      int boundingWidth = (int) obstacleBoundingBox.getWidth();
      int boundingHeight = (int) obstacleBoundingBox.getHeight();

      for (int x = boundingX; x <= boundingX + boundingWidth; x++) {
        for (int y = boundingY; y <= boundingY + boundingHeight; y++) {
          if (obstacle.contains(x, y)) {
            field[x][y].obstaclize();
          }
        }
      }
    }
  }

  /**
   * Finds the GridBox at the designated coordinates (a Translation2d).
   *
   * @param p The point at which the GridBox is to be found.
   * @return The GridBox at those coordinates
   */
  public GridBox coordsToBox(Translation2d p) {
    int translatedX = (int) p.getX() / GRID_SIDE_LENGTH;
    int translatedY = (int) p.getY() / GRID_SIDE_LENGTH;
    return fullField[translatedX][translatedY];
  }

  /** Removes the temporary obstacles from the field. */
  public void resetTemps() {
    fullField = baseField;
    movingObstacles.clear();
  }

  /**
   * @return The current moving obstacles on the field.
   */
  public List<Obstacle> getMovingObstacles() {
    return movingObstacles;
  }

  /**
   * @return The GridBox double array representing the field.
   */
  public GridBox[][] field() {
    return fullField;
  }
}
