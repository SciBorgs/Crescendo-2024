package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.OFFSET;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.pivot.PivotConstants;

public class NoteVisualizer implements Logged {
  @Log.NT private static Pose3d[] notes = new Pose3d[] {};
  private static Supplier<Pose2d> pose = Pose2d::new;
  private static Supplier<Rotation2d> angle = () -> PivotConstants.STARTING_ANGLE;
  private static DoubleSupplier velocity = () -> 1;

  public static void setSuppliers(
      Supplier<Pose2d> robotPose, Supplier<Rotation2d> pivotAngle, DoubleSupplier shotVelocity) {
    pose = robotPose;
    angle = pivotAngle;
    velocity = shotVelocity;
  }

  @Log.NT
  public static void logNotes() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    StructArrayTopic<Pose3d> topic = inst.getStructArrayTopic("poses", Pose3d.struct);
    StructArrayPublisher<Pose3d> pub = topic.publish();
    pub.set(notes);
  }

  public static Command shoot() {
    return new ScheduleCommand(
            Commands.defer(
                () -> {
                  final Pose3d notePose =
                      new Pose3d(pose.get())
                          .plus(
                              new Transform3d(
                                  OFFSET,
                                  new Rotation3d(
                                      0,
                                      angle.get().getRadians(),
                                      pose.get().getRotation().getRadians())));
                  final Translation3d speakerPose = getSpeaker();

                  final double time =
                      notePose.getTranslation().getDistance(speakerPose) / velocity.getAsDouble();
                  final Timer timer = new Timer();

                  timer.start();

                  return Commands.run(
                          () -> {
                            notes =
                                new Pose3d[] {
                                  notePose.interpolate(
                                      new Pose3d(speakerPose, new Rotation3d()), timer.get() / time)
                                };
                          })
                      .until(() -> timer.hasElapsed(time))
                      .finallyDo(
                          () -> {
                            notes = new Pose3d[] {};
                          });
                },
                Set.of()))
        .ignoringDisable(true);
  }
}
