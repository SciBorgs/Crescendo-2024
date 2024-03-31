package org.sciborgs1155.robot.pathfinding;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class GridBox {
  private boolean obstacled = false;

  private int assignedValue = 0;

  private double cost = 0;

  private int XValue;
  private int YValue;

  // List of GridBoxes that this GridBox will not take new costs from.
  private List<GridBox> ignoreList = new ArrayList<>(List.of());

  /**
   * Constructor for a GridBox.
   *
   * @param XValue
   * @param YValue
   */
  public GridBox(int XValue, int YValue) {
    this.XValue = XValue;
    this.YValue = YValue;
  }

  /**
   * Sets the cost of the GridBox.
   *
   * @param p the point from which the proposed cost is being taken from.
   * @param proposed The proposed number to be set as the cost.
   * @param reset A boolean. If true, the cost will be set to proposed. If false, the cost will be
   *     set to the minimum of the current cost and the proposed cost.
   */
  public void setCost(GridBox p, double proposed, boolean reset) {
    ignoreList.add(p);
    cost = reset ? proposed : Math.min(cost, proposed);
    assignedValue++;
  }

  /** Resets the cost, assigned value and ignore list of the GridBox. */
  public void reset() {
    cost = 0;
    assignedValue = 0;
    ignoreList.clear();
  }

  /**
   * @return The cost of the GridBox.
   */
  public double getCost() {
    return cost;
  }

  /**
   * Resets the obstacled state of the GridBox, meaning that it will be considered as "not an
   * obstacle"
   */
  public void deobstaclize() {
    obstacled = false;
  }

  /**
   * Turns the obstacled status of the GridBox to true, meaning that it will be considered as "an
   * obstacle"
   */
  public void obstaclize() {
    obstacled = true;
  }

  /**
   * @return Whether the GridBox is considered an obstacle or not.
   */
  public boolean checkObstacled() {
    return obstacled;
  }

  public boolean offEdge() {
    return getX() != 0
        && getY() != 0
        && getX() != GriddedField.LENGTH_GRID_NUMBER - 1
        && getY() != GriddedField.LENGTH_GRID_NUMBER - 1;
  }

  /**
   * @return The X value of the GridBox.
   */
  public int getX() {
    return XValue;
  }

  /**
   * @return The Y value of the GridBox.
   */
  public int getY() {
    return YValue;
  }

  /**
   * @return The X-value of the GridBox outside of the grid.
   */
  public double getUngriddedX() {
    return XValue * PathfindingConstants.GRID_SIDE_LENGTH.in(Centimeters);
  }

  /**
   * @return The Y-value of the GridBox outside of the grid.
   */
  public double getUngriddedY() {
    return YValue * PathfindingConstants.GRID_SIDE_LENGTH.in(Centimeters);
  }

  /**
   * Converts the GridBox to a Translation2d.
   *
   * @return The Translation2d position of the GridBox.
   */
  public Translation2d boxToTranslation() {
    return new Translation2d(getUngriddedX(), getUngriddedY());
  }

  /**
   * Finds whether or not the grid should have its neighbor's cost considered. This method mainly
   * exists for performance reasons.
   *
   * @param p The gridbox from which the proposed new cost is coming from.
   * @return Whether or not the grid will take into accound its neighbor's cost.
   */
  public boolean assignable(GridBox p) {
    return assignedValue <= PathfindingConstants.MAX_ASSIGNED_VALUE
        && !(ignoreList.contains(p))
        && !obstacled;
  }
}
