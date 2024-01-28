package org.sciborgs1155.robot.shooter.pivot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.sciborgs1155.robot.Constants.Dimensions;

public class PivotVisualizer {
  private final MechanismLigament2d arm;
  private static int instance = 0;

  public PivotVisualizer(Mechanism2d mech, Color8Bit color) {
    MechanismRoot2d chassis =
        mech.getRoot("Chassis" + instance, Dimensions.BASE_OFFSET, Dimensions.BASE_HEIGHT);
    arm =
        chassis.append(
            new MechanismLigament2d("Arm" + instance, Dimensions.SHOOTER_ARM_LENGTH, 0, 4, color));
    instance++;
  }

  public void setState(double angle) {
    arm.setAngle(angle);
  }
}
