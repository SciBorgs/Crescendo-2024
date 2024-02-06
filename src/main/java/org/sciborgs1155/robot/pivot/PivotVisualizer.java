package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.sciborgs1155.robot.shooter.ShooterConstants;

public class PivotVisualizer {
  private final MechanismLigament2d arm;
  private static int instance = 0;

  public PivotVisualizer(Mechanism2d mech, Color8Bit color) {
    MechanismRoot2d chassis =
        mech.getRoot(
            "Chassis" + instance, PivotConstants.OFFSET.getX(), PivotConstants.OFFSET.getY());
    arm =
        chassis.append(
            new MechanismLigament2d(
                "Arm" + instance, ShooterConstants.LENGTH.in(Meters), 0, 4, color));
    instance++;
  }

  public void setState(Rotation2d angle) {
    arm.setAngle(angle);
  }
}
