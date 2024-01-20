package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import org.sciborgs1155.robot.drive.DriveConstants;

/**
 * Constants is a globally accessible class for storing immutable values. Every value should be
 * <code>public static final</code>.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * @see Units
 */
public class Constants {
  public static final Measure<Time> PERIOD = Seconds.of(0.02); // roborio tickrate (s)
  public static final double DEADBAND = 0.1;
  public static final double MAX_RATE =
      DriveConstants.MAX_ACCEL.baseUnitMagnitude()
          / DriveConstants.MAX_ANGULAR_SPEED.baseUnitMagnitude();
  public static final double SLOW_SPEED = 0.33;
  public static final double FULL_SPEED = 1.0;
}
