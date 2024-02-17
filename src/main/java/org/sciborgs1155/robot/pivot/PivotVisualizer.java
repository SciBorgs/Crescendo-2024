package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class PivotVisualizer implements Sendable, AutoCloseable {
  private final Mechanism2d mech;
  private final MechanismLigament2d arm;

  public PivotVisualizer(Color8Bit color) {
    mech = new Mechanism2d(2, 2);
    MechanismRoot2d chassis = mech.getRoot("Chassis", 1 + OFFSET.getX(), OFFSET.getY());
    arm = chassis.append(new MechanismLigament2d("Arm", LENGTH.in(Meters), 0, 4, color));
  }

  public void setState(double angle) {
    arm.setAngle(Units.radiansToDegrees(angle + Math.PI));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    mech.initSendable(builder);
  }

  @Override
  public void close() throws Exception {
    mech.close();
  }
}
