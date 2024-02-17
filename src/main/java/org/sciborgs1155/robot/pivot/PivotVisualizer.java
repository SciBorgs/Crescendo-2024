package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.pivot.PivotConstants.OFFSET;
import static org.sciborgs1155.robot.pivot.Pivot.*;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class PivotVisualizer {
  private static Pose3d[] setpoint = new Pose3d[]{};
  private static Supplier<Translation3d> pose = ()-> OFFSET;
  private static Supplier<Rotation3d> angle = () -> PivotConstants.STARTING_ANGLE;
  private static Pose3d measurement;
  private static DoubleSupplier velocity = () -> 1;
  private static StructArrayPublisher<Pose3d> publisher;
  private static double yVelocity;
  private static final double g = 9.81;


 public static void setSuppliers(Supplier<Rotation3d> pivotAngle, Supplier<Translation3d> pivotPose, DoubleSupplier pivotVelocity) {
    angle = pivotAngle;
    pose = pivotPose;
    velocity = pivotVelocity;
  }
  
 public static void startPublishing() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    StructArrayTopic<Pose3d> topic = inst.getStructArrayTopic("angles", Pose3d.struct);
    publisher = topic.publish();
  }

  public static void logPivot() {
    publisher.set(setpoint);
  }

    public static Command shoot() {
    return new ScheduleCommand(
            Commands.defer(
                () -> {
                  measurement =
                      new Pose3d(pose.get(), angle.get())
                          .plus(
                              new Transform3d(
                                  OFFSET, new Rotation3d(0, angle.get().getRadians(), 0)));
                  yVelocity = velocity.getAsDouble() * angle.get().getSin();

                  return Commands.run(
                          () -> {
                            setpoint = new Pose3d[] {measurement};
                            yVelocity -= g * PERIOD.in(Seconds);
                          })
                      .until(() -> measurement.getZ() < 0.0)
                      .finallyDo(
                          () -> {
                            setpoint = new Pose3d[] {};
                          });
                }, Set.of()))
        .ignoringDisable(true);
  }
}