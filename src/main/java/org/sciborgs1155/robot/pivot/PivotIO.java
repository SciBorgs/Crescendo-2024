package org.sciborgs1155.robot.pivot;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import monologue.Logged;

/** Represents the hardware of a pivot. */
public interface PivotIO extends AutoCloseable, Logged {
  /**
   * Sets the pivot's input voltage.
   *
   * @param voltage The input voltage.
   */
  public void setVoltage(double voltage);

  /**
   * Sets the current limit of the pivot.
   *
   * @param limit The current limit. It should be <b>CURRENT_LIMIT</b> for regular pivot, and
   *     <b>CLIMBER_CURRENT_LIMIT</b> for climbing pivot.
   */
  public void currentLimit(Measure<Current> limit);

  /**
   * Returns the current position of the pivot.
   *
   * @return The current position of the pivot in radians.
   */
  public double getPosition();

  /**
   * Returns the current velocity of the pivot.
   *
   * @return The current velocity of pivot in radians / second.
   */
  public double getVelocity();
}
