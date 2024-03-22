package org.sciborgs1155.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtils {
  /**
   * Modifies setpoints so they don't go over the max acceleration, while attempting to renormalize
   * the speeds.
   *
   * <p>Sometimes a setpoint which exceeds the acceleration from the previous setpoint is passed, so
   * this method ensures no individual modules do so. Ideally we would renormalize the speeds, but
   * this is not always possible depending on the situation. Therefore, we clamp for proportionality
   * between the max acceleration possible in either direction.
   *
   * @param moduleStates the pre-transformed setpoints.
   * @param prevModuleStates the setpoints previously passed to the robot (post-transformed).
   * @param maxAcceleration the acceleration used to limit the setpoints.
   * @param deltaTime the change in time from the last iteration / call to this method.
   */
  public static void desaturateWheelAcceleration(
      SwerveModuleState[] moduleStates,
      SwerveModuleState[] prevModuleStates,
      double maxAcceleration,
      double deltaTime) {
    double minOffendingAcceleration = Double.POSITIVE_INFINITY;
    double associatedVelocity = 0;
    double associatedPrevVelocity = 0;
    for (int i = 0; i < moduleStates.length; i++) {
      double currAcceleration =
          (moduleStates[i].speedMetersPerSecond - prevModuleStates[i].speedMetersPerSecond)
              / deltaTime;
      if (Math.abs(currAcceleration) < maxAcceleration) {
        continue;
      }
      if (Math.abs(currAcceleration) < Math.abs(minOffendingAcceleration)) {
        minOffendingAcceleration = currAcceleration;
        associatedVelocity = moduleStates[i].speedMetersPerSecond;
        associatedPrevVelocity = prevModuleStates[i].speedMetersPerSecond;
      }
    }
    if (minOffendingAcceleration == Double.POSITIVE_INFINITY) {
      return;
    }
    if (associatedVelocity == 0) {
      associatedVelocity = 0.000000001;
    }
    double proportionFactor =
        (associatedPrevVelocity
                + MathUtil.clamp(
                    minOffendingAcceleration * deltaTime,
                    -maxAcceleration * deltaTime,
                    maxAcceleration * deltaTime))
            / associatedVelocity;
    if (Math.abs(minOffendingAcceleration) > maxAcceleration) {
      for (int i = 0; i < moduleStates.length; i++) {
        moduleStates[i].speedMetersPerSecond =
            MathUtil.clamp(
                moduleStates[i].speedMetersPerSecond * proportionFactor,
                prevModuleStates[i].speedMetersPerSecond - maxAcceleration * deltaTime,
                prevModuleStates[i].speedMetersPerSecond + maxAcceleration * deltaTime);
      }
    }
    System.out.println();
  }
}
