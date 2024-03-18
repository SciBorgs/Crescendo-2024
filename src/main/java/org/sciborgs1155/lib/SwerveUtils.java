package org.sciborgs1155.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtils {
  public static void desaturateWheelAcceleration(
      SwerveModuleState[] moduleStates,
      SwerveModuleState[] prevModuleStates,
      double maxAcceleration,
      double deltaTime) {
    for (int i = 0; i < moduleStates.length; i++) {
      if (moduleStates[i].angle.getDegrees() < 0) {
        moduleStates[i].angle = new Rotation2d(Math.abs(moduleStates[i].angle.getRadians()));
        moduleStates[i].speedMetersPerSecond *= -1;
      }
    }
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
        System.out.println("prev: " + prevModuleStates[i].speedMetersPerSecond);
        System.out.println("pre-transform: " + moduleStates[i].speedMetersPerSecond);
        moduleStates[i].speedMetersPerSecond =
            MathUtil.clamp(
                moduleStates[i].speedMetersPerSecond * proportionFactor,
                prevModuleStates[i].speedMetersPerSecond - maxAcceleration * deltaTime,
                prevModuleStates[i].speedMetersPerSecond + maxAcceleration * deltaTime);
        
        System.out.println("post-transform: " + moduleStates[i].speedMetersPerSecond);
      }
    }
    System.out.println();
  }
}