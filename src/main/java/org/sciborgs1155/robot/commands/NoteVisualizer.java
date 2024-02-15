package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.OFFSET;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;
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
import monologue.Logged;
import org.sciborgs1155.robot.pivot.PivotConstants;

public class NoteVisualizer implements Logged {
  private static Pose3d[] notes = new Pose3d[] {};
  private static Supplier<Pose2d> pose = Pose2d::new;
  private static Supplier<Rotation2d> angle = () -> PivotConstants.STARTING_ANGLE;
  private static DoubleSupplier velocity = () -> 1;

  private static double yVelocity;

  private static final double g = 9.81;

  public static void setSuppliers(
      Supplier<Pose2d> robotPose, Supplier<Rotation2d> pivotAngle, DoubleSupplier shotVelocity) {
    pose = robotPose;
    angle = pivotAngle;
    velocity = shotVelocity;
  }

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

                  final double xVelocity =
                      velocity.getAsDouble() * Math.cos(angle.get().getRadians());
                  yVelocity = velocity.getAsDouble() * Math.sin(angle.get().getRadians());

                  // time for note to reach initial height after arc
                  final double time = 2.0 * Math.sin(velocity.getAsDouble()) / g;
                  final Timer timer = new Timer();

                  timer.start();

                  return Commands.run(
                          () -> {
                            notes =
                                new Pose3d[] {
                                  notePose.exp( // x (fwd), y (l, r), z (up & down)
                                      new Twist3d(
                                          xVelocity * PERIOD.in(Seconds),
                                          0,
                                          yVelocity * PERIOD.in(Seconds),
                                          0,
                                          0,
                                          0))
                                };
                            yVelocity -= g * PERIOD.in(Seconds);
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
