package org.sciborgs1155.lib;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtils {
    public static void desaturateWheelAcceleration(
      SwerveModuleState[] moduleStates, SwerveModuleState[] prevModuleStates, double attainableMaxAcceleration) {
    // acceleration we give it that is potentially bad
    double givenMaxAcceleration = 0;
    for (int i = 0; i < moduleStates.length; i++) {
      givenMaxAcceleration =
          Math.max(
              givenMaxAcceleration,
              Math.abs(
                  moduleStates[i].speedMetersPerSecond
                      - prevModuleStates[i].speedMetersPerSecond));
    }
    if (givenMaxAcceleration > attainableMaxAcceleration) {
      for (int i = 0; i < moduleStates.length; i++) {
        moduleStates[i].speedMetersPerSecond =
            (moduleStates[i].speedMetersPerSecond - prevModuleStates[i].speedMetersPerSecond)
                    / givenMaxAcceleration
                    * attainableMaxAcceleration
                + prevModuleStates[i].speedMetersPerSecond;
      }
    }
  }
}
