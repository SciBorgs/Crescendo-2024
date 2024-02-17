package org.sciborgs1155.robot.pivot;

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
