package org.sciborgs1155.robot.shooter;

import monologue.Logged;

public interface WheelIO extends AutoCloseable, Logged {
  /**
   * Sets the voltage for the flywheel.
   *
   * @param voltage The voltage
   */
  void setVoltage(double voltage);

  /**
   * The velocity of the flywheel, in radians per seconds.
   *
   * @return The velocity of the flywheel, in radians per seconds.
   */
  double velocity();
}
