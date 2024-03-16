package org.sciborgs1155.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtils {
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
  }
}
