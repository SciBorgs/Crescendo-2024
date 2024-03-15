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
    double associatedSpeed = 0;
    double associatedPrevSpeed = 0;
    for (int i = 0; i < moduleStates.length; i++) {
      double currAcceleration =
      Math.abs(moduleStates[i].speedMetersPerSecond
              - prevModuleStates[i].speedMetersPerSecond / deltaTime);
      if (currAcceleration < attainableMaxAcceleration) {
        continue;
      }
      if (currAcceleration < givenMaxAcceleration) {
        givenMaxAcceleration = currAcceleration;
        associatedSpeed = moduleStates[i].speedMetersPerSecond;
        associatedPrevSpeed = prevModuleStates[i].speedMetersPerSecond;
      }
    }
    if (associatedSpeed == 0) {
      associatedSpeed = 0.000000000001;
    }
    if (givenMaxAcceleration == Double.POSITIVE_INFINITY) {
      return;
    }
    System.out.println(associatedSpeed);
    double factor =
        (associatedPrevSpeed
                + MathUtil.clamp(
                    givenMaxAcceleration, -attainableMaxAcceleration, attainableMaxAcceleration))
            / associatedSpeed;
    System.out.println(factor);
    if (givenMaxAcceleration > attainableMaxAcceleration) {
      for (int i = 0; i < moduleStates.length; i++) {
        moduleStates[i].speedMetersPerSecond = moduleStates[i].speedMetersPerSecond * factor;
      }
    }
  }
}
