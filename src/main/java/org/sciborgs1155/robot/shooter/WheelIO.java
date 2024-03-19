package org.sciborgs1155.robot.shooter;

import monologue.Logged;

public interface WheelIO extends AutoCloseable, Logged {
  void setVoltage(double voltage);

  double velocity();

  void setInverted(boolean inverted);
}
