package org.sciborgs1155.robot.pathfinding;

import static org.sciborgs1155.robot.pathfinding.PathfindingConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.robot.pathfinding.Convenience.Point;

/**
 * (jacob showed me how to do this)
 *
 * @author Charlie Kerr
 */
public class CharliesAstar {

  private GriddedField field;

  // The setpoint is the position of the robot, not the goal of the robot.
  private GridBox setPoint;

  // The set of translations to adjacent squares. Starts at the right, rotates counter-clockwise.
  public static final Point[] intpair = {
    new Point(1, 0),
    new Point(1, 1),
    new Point(0, 1),
    new Point(-1, 1),
    new Point(-1, 0),
    new Point(-1, -1),
    new Point(0, -1),
    new Point(1, -1)
  };

  // The set of translations which move diagonally.
  public static final List<Point> diagBoxes =
      List.of(new Point(1, 1), new Point(-1, 1), new Point(-1, -1), new Point(1, -1));

  /**
   * Constructor for the A star algorithm.
   *
   * @param field The field on which the algorithm is working on.
   * @param setPoint The position of the ROBOT. Keep in mind that this is !!NOT!! the GOAL!
   */
  public CharliesAstar(GriddedField field, GridBox setPoint) {
    this.field = field;
    this.setPoint = setPoint;
  }

  /**
   * Finds the angle, as a Rotation2d, from one GridBox to the setpoint.
   *
   * @param currentBox The starting point of the vector that the angle will be based off of.
   * @return A Rotation2d with the angle from the vector created.
   */
  public Rotation2d pointDirection(GridBox currentBox) {
    return new Translation2d(
            setPoint.getX() - currentBox.getX(), setPoint.getY() - currentBox.getY())
        .getAngle();
  }

  /**
   * Shortcut to whether the movement is diagonal. Will only work if given items from
   * CharliesAstar.intpair. This method exists for optimization reasons. The long way around this
   * problem is to calculate the angle and then determine whether it is diagonal from there.
   *
   * @param p A Point from intpair.
   * @return If the point is in CharliesAstar.diagBoxes.
   */
  private static boolean diagonalMovement(Point p) {
    return diagBoxes.contains(p);
  }

  private static double findCost(Point p) {
    return diagonalMovement(p) ? 1.4 : 1;
    /* 1.4 is an approximation for the square root of two.
     * Extreme precision is not needed especially with the amount
     * of calculations per second using this number. */
  }

  /**
   * Returns three pairs of integers, with them representing the translation needed to get the three
   * boxes in the general direction of the rotation.
   *
   * @param rotation The rotation inputted to get the three boxes
   * @return Three pairs of integers which represent (x,y) translation to the three boxes
   */
  public static Point[] threeRelevantCoords(Rotation2d rotation, boolean reverse) {
    /* Using degrees because it's more memory efficient than an
     * irrational decimal (multiple of pi) */
    double degrees = rotation.getDegrees();

    /* Converting to an int rounds down. This gives us the general rotation
     * (right = 0, top right = 1, top left = 3, etc) */
    int directionChange = ((int) (degrees + 22.5) / 45);

    // Assigning the three values to result. Sadly, ternary operators don't work with arrays.
    Point[] result;
    if (!reverse) {
      result =
          new Point[] {
            intpair[directionChange % 8],
            intpair[(directionChange - 1) % 8],
            intpair[(directionChange + 1) % 8]
          };
    } else {
      result =
          new Point[] {
            intpair[(directionChange + 4) % 8],
            intpair[(directionChange + 3) % 8],
            intpair[(directionChange + 5) % 8]
          };
    }

    return result;
  }

  /**
   * Returns five pairs of integers, with them representing the translation needed to get the five
   * boxes in the general direction of the rotation.
   *
   * @param rotation The rotation inputted to get the five boxes.
   * @return Five pairs of integers which represent (x,y) translation to the five boxes.
   */
  public static Point[] fiveRelevantCoords(Rotation2d rotation, boolean reverse) {
    /* Using degrees because it's more memory efficient than an
     * irrational decimal (multiple of pi) */
    double degrees = rotation.getDegrees();

    /* Converting to an int rounds down. This gives us the general rotation
     * (right = 0, top right = 1, top left = 3, etc) */
    int directionChange = ((int) (degrees + 22.5) / 45);

    // Assigning the three values to result. Sadly, ternary operators don't work with arrays.
    Point[] result;
    if (!reverse) {
      result =
          new Point[] {
            intpair[directionChange % 8],
            intpair[(directionChange - 1) % 8],
            intpair[(directionChange + 1) % 8],
            intpair[(directionChange + 2) % 8],
            intpair[(directionChange - 2) % 8]
          };
    } else {
      result =
          new Point[] {
            intpair[(directionChange + 4) % 8],
            intpair[(directionChange + 3) % 8],
            intpair[(directionChange + 5) % 8],
            intpair[(directionChange + 2) % 8],
            intpair[(directionChange + 6) % 8]
          };
      // Not very beautiful code, is it? Unfortunately, it is the most effective.
    }
    return result;
  }

  /**
   * Assigns the starting point to one grid box, initiating a chain reaction and a positive feedback
   * loop to assign nearly all GridBoxes a number relating to their supposed pathway.
   *
   * @param startingPoint The goal of the robot. It is labeled "Starting point," however this is the
   *     starting point of the ALGORITHM, NOT the ROBOT. Input the starting point as the robot's
   *     goal.
   */
  public void firstCostAssign(GridBox startingPoint) {
    startingPoint.setCost(startingPoint, 0, true);
    assignCosts(startingPoint);
  }

  /**
   * The chain reaction for the cost assignment. This utilizes recursion to make sure that no
   * important box is left unassigned. After every box is assigned, it assigns a certain amount of
   * boxes around them. If the boxes are past a certain distance from the starting point, they will
   * begin to only assign costs to boxes in front of them in the direction of the set point, so that
   * unimportant boxes in the corner are not assigned, as a method of optimization.
   *
   * @param box The box which is currently being assigned.
   */
  public void assignCosts(GridBox box) {
    // Direction for the directional box methods.
    Rotation2d rotation = pointDirection(box);

    if (box != setPoint && box.offEdge()) {
      if (box.getCost() < COST_CREATIVITY) { // If the current box is within the current radius.
        for (Point i : intpair) {

          // Getting a box adjacent to the inputted one.
          GridBox proposedBox = field.field()[box.getX() + i.getX()][box.getY() + i.getY()];

          // Setting the cost of the adjacent box.
          box.setCost(proposedBox, proposedBox.getCost() + findCost(i), false);
          if (proposedBox.assignable(box)) {
            /* So long as this proposed box is assignable, the method will recur
             * with this new proposed box as the input. */
            assignCosts(proposedBox);
          }
        }
      } else { // If the current box is not within the current radius.
        for (Point i : fiveRelevantCoords(rotation, true)) {

          // Getting an adjacent box.
          GridBox proposedBox = field.field()[box.getX() + i.getX()][box.getY() + i.getY()];
          box.setCost(proposedBox, proposedBox.getCost() + findCost(i), false);
        }

        for (Point i : threeRelevantCoords(rotation, false)) {

          // Getting an adjacent box.
          GridBox proposedBox = field.field()[box.getX() + i.getX()][box.getY() + i.getY()];

          if (proposedBox.assignable(box)) {
            /* So long as this proposed box is assignable, the method will recur
             * with this new proposed box as the input. */
            assignCosts(proposedBox);
          }
        }
      }
    }
  }

  /**
   * Finds which box, adjacent to the inputted one, has the lowest cost. Includes diagonals.
   *
   * @param box The inputted box.
   * @return The box adjacent to the inputted box, of which has the lowest cost of all adjacent
   *     boxes.
   */
  private GridBox lowestCostNearby(GridBox box) {
    GridBox lowest = box;
    for (Point point : intpair) {
      if (box.getX() + point.getX() >= 0
          && box.getX() + point.getX() < GriddedField.LENGTH_GRID_NUMBER
          && box.getY() + point.getY() >= 0
          && box.getY() + point.getY() < GriddedField.WIDTH_GRID_NUMBER) {
        GridBox proposedBox = field.field()[box.getX() + point.getX()][box.getY() + point.getY()];
        if (!proposedBox.checkObstacled()) {
          lowest = proposedBox;
        }
      }
    }
    return lowest;
  }

  /**
   * Creates a list of Translation2ds representing the calculated path from the start point to the
   * end point. This method assumes that the GridBoxes have already been assigned costs.
   *
   * @param pathStart The start point of the path.
   * @param pathEnd The end point of the path.
   * @return A Translation2d list with the points in the path from pathStart to pathEnd.
   */
  public List<Translation2d> pathMaker(Translation2d pathStart, Translation2d pathEnd) {
    // Creating the list
    List<Translation2d> points = new ArrayList<>(List.of(pathStart));

    // Appending to the list
    while (!points.contains(pathEnd)) {
      points.add(
          lowestCostNearby(field.coordsToBox(points.get(points.size() - 1))).boxToTranslation());
    }
    return points;
  }

  /**
   * Finds the next point in the path from the pathStart to the pathEnd. This method assumes that
   * the GridBoxes have already been assigned costs.
   *
   * @param currentPos The start point of the path.
   * @return The next Translation2d in the path.
   */
  public Translation2d nextPos(Translation2d currentPos) {
    return lowestCostNearby(field.coordsToBox(currentPos)).boxToTranslation();
  }

  /**
   * @return The current setPoint of the A* algorithm.
   */
  public GridBox getSetPoint() {
    return setPoint;
  }

  /**
   * Changes the setPoint.
   *
   * @param newBox The new setPoint.
   */
  public void changeSetPoint(GridBox newBox) {
    setPoint = newBox;
  }
}
