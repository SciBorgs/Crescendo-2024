package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Generalized gyroscope. Pigeon2, Navx, and SimGyro are to be implemented */
public interface GyroIO extends AutoCloseable {
  /** Calibrates the gyroscope. Pigeon2 does not need to do anything here. */
  public default void calibrate() {}

  /** Returns the rate of rotation. */
  public double getRate();

  /** Returns the heading of the robot as a Rotation2d. */
  public default Rotation2d getRotation2d() {
    return getRotation3d().toRotation2d();
  }

  /** Returns the heading of the robot as a Rotation3d. */
  public Rotation3d getRotation3d();

  /** Resets heading to 0 */
  public void reset();
}
