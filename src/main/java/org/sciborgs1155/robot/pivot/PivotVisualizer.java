package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class PivotVisualizer {
  private final MechanismLigament2d arm;
  private static int instance = 0;

  public PivotVisualizer(Mechanism2d mech, Color8Bit color) {
    MechanismRoot2d chassis = mech.getRoot("Chassis" + instance, 1 + OFFSET.getX(), OFFSET.getY());
    arm = chassis.append(new MechanismLigament2d("Arm" + instance, LENGTH.in(Meters), 0, 4, color));
    instance++;
  }

  public void setState(Rotation2d angle) {
    arm.setAngle(angle.rotateBy(Rotation2d.fromRadians(Math.PI)));
  }
}
