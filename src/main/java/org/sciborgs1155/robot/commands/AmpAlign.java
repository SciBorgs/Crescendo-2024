package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.pivot.PivotConstants.PRESET_AMP_ANGLE;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Constants.Field;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.pivot.Pivot;

public class AmpAlign {
  private final Drive drive;
  private final Pivot pivot;
  private final Shooting shooting;

  public AmpAlign(Drive drive, Pivot pivot, Shooting shooting) {
    this.drive = drive;
    this.pivot = pivot;
    this.shooting = shooting;
  }

  public Rotation2d findAmpAngle() {
    return Field.ampCoordinates().getRotation();
  }

  /**
   * Turns the robot such that the shooter is pointing into the amp opening. Additionally, while
   * held, moves the pivot arm to the required angle for the note to go into the amp.
   */
  public Command snapToAmp(DoubleSupplier vx, DoubleSupplier vy) {
    return drive.driveTo(null) // do math ah
        .andThen(drive.drive(vx, vy, this::findAmpAngle));
  }

  public Command shootAmp() {
    return pivot
        .runPivot(PRESET_AMP_ANGLE)
        .andThen(shooting.shoot(RadiansPerSecond.of(11.55))); // CURRENTLY A MADE UP NUMBER
  }
}
