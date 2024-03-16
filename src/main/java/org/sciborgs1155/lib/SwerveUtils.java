package org.sciborgs1155.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtils {
  public static void desaturateWheelAcceleration(
      SwerveModuleState[] moduleStates,
      SwerveModuleState[] prevModuleStates,
      double attainableMaxAcceleration,
      double deltaTime) {
    // acceleration we give it that is potentially bad
    double givenMaxAcceleration = Double.POSITIVE_INFINITY;
    double associtatedVelocity = 0;
    double associatedPrevVelocity = 0;
    for (int i = 0; i < moduleStates.length; i++) {
      double currAcceleration =
              (moduleStates[i].speedMetersPerSecond
                  - prevModuleStates[i].speedMetersPerSecond) / deltaTime;
      if (Math.abs(currAcceleration) < Math.abs(attainableMaxAcceleration)) {
        continue;
      }
      if (Math.abs(currAcceleration) < Math.abs(givenMaxAcceleration)) {
        givenMaxAcceleration = currAcceleration;
        associtatedVelocity = moduleStates[i].speedMetersPerSecond;
        associatedPrevVelocity = prevModuleStates[i].speedMetersPerSecond;
      }
    }
    if (associtatedVelocity == 0) {
      associtatedVelocity = 0.000000000001;
    }
    if (givenMaxAcceleration == Double.POSITIVE_INFINITY) {
      return;
    }
    double factor =
        (associatedPrevVelocity
                + MathUtil.clamp(
                    givenMaxAcceleration, -attainableMaxAcceleration, attainableMaxAcceleration))
            / associtatedVelocity;
    if (givenMaxAcceleration > attainableMaxAcceleration) {
      for (int i = 0; i < moduleStates.length; i++) {
        moduleStates[i].speedMetersPerSecond =
            MathUtil.clamp(
                moduleStates[i].speedMetersPerSecond * factor,
                prevModuleStates[i].speedMetersPerSecond - attainableMaxAcceleration,
                prevModuleStates[i].speedMetersPerSecond + attainableMaxAcceleration);
      }
    }
  }
}
