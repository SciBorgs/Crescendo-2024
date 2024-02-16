package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.Constants.Field.*;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Logged;
import org.sciborgs1155.robot.pivot.PivotConstants;

public class NoteVisualizer implements Logged {
  private static Pose3d[] firedNotes = new Pose3d[] {};
  private static Pose3d[] externalNotes = new Pose3d[] {};
  private static Supplier<Pose2d> pose = Pose2d::new;
  private static Supplier<Rotation2d> angle = () -> PivotConstants.STARTING_ANGLE;
  private static DoubleSupplier velocity = () -> 1;

  private static double yVelocity;
  private static Pose3d currentNotePose;

  private static final double g = 9.81;
  private static StructArrayPublisher<Pose3d> publisher;

  public static void setSuppliers(
      Supplier<Pose2d> robotPose, Supplier<Rotation2d> pivotAngle, DoubleSupplier shotVelocity) {
    pose = robotPose;
    angle = pivotAngle;
    velocity = shotVelocity;
  }

  /** Set up NT publisher. Call only once before beginning to log notes. */
  public static void startPublishing() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    StructArrayTopic<Pose3d> topic = inst.getStructArrayTopic("poses", Pose3d.struct);
    publisher = topic.publish();
  }

  public static void logNotes() {
    publisher.set(firedNotes);
  }

  public static Command shoot() {
    return new ScheduleCommand(
            Commands.defer(
                () -> {
                  currentNotePose =
                      new Pose3d(pose.get())
                          .plus(
                              new Transform3d(
                                  OFFSET, new Rotation3d(0, angle.get().getRadians(), 0)));

                  final double xVelocity =
                      Math.abs(velocity.getAsDouble() * angle.get().times(-1).getCos());
                  yVelocity = velocity.getAsDouble() * angle.get().getSin();

                  return Commands.run(
                          () -> {
                            currentNotePose =
                                currentNotePose.exp( // x (fwd), y (l, r), z (up & down)
                                    new Twist3d(
                                        xVelocity * PERIOD.in(Seconds),
                                        0,
                                        yVelocity * PERIOD.in(Seconds),
                                        0,
                                        0,
                                        0));
                            firedNotes = new Pose3d[] {currentNotePose};
                            yVelocity -= g * PERIOD.in(Seconds);
                          })
                      .until(() -> currentNotePose.getZ() < 0.0)
                      .finallyDo(
                          () -> {
                            firedNotes = new Pose3d[] {};
                          });
                },
                Set.of()))
        .ignoringDisable(true);
  }
}
