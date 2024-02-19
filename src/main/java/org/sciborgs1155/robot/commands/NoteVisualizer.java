package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.Constants.Field.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Logged;
import org.sciborgs1155.robot.shooter.ShooterConstants;

public class NoteVisualizer implements Logged {
  // notes
  private static LinkedList<Pose3d> notes =
      new LinkedList<>(
          List.of(
              BLUE_LEFT_NOTE,
              BLUE_MID_NOTE,
              BLUE_RIGHT_NOTE,
              CENTER_NOTE_ONE,
              CENTER_NOTE_TWO,
              CENTER_NOTE_THREE,
              CENTER_NOTE_FOUR,
              CENTER_NOTE_FIVE,
              RED_LEFT_NOTE,
              RED_MID_NOTE,
              RED_RIGHT_NOTE));
  private static boolean carryingNote = false;
  private static ArrayList<Pose3d> pathPosition = new ArrayList<>();

  // suppliers
  private static Supplier<Pose2d> pose = Pose2d::new;
  private static Supplier<Rotation3d> angle = Rotation3d::new;
  private static DoubleSupplier velocity = () -> 1;

  private static double zVelocity;
  private static int i = 0;

  private static Pose3d currentNotePose = new Pose3d();
  private static Pose3d lastNotePose = new Pose3d();

  // publishers
  private static StructPublisher<Pose3d> shotNotePub;
  private static StructArrayPublisher<Pose3d> pathPub;
  private static StructArrayPublisher<Pose3d> notesPub;
  private static BooleanPublisher hasNotePub;

  public static void setSuppliers(
      Supplier<Pose2d> robotPose, Supplier<Rotation3d> pivotAngle, DoubleSupplier shotVelocity) {
    pose = robotPose;
    angle = pivotAngle;
    velocity = shotVelocity;
  }

  /** Set up NT publisher. Call only once before beginning to log notes. */
  public static void startPublishing() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    StructArrayTopic<Pose3d> notesTopic = inst.getStructArrayTopic("notes", Pose3d.struct);
    StructArrayTopic<Pose3d> notePathTopic = inst.getStructArrayTopic("note path", Pose3d.struct);
    BooleanTopic hasNoteTopic = inst.getBooleanTopic("holding note");
    StructTopic<Pose3d> shotNoteTopic = inst.getStructTopic("shot note", Pose3d.struct);

    notesPub = notesTopic.publish();
    pathPub = notePathTopic.publish();
    hasNotePub = hasNoteTopic.publish();
    shotNotePub = shotNoteTopic.publish();

    notesPub.set(notes.toArray(new Pose3d[0]));
  }

  public static void log() {
    shotNotePub.set(currentNotePose);
    hasNotePub.set(carryingNote);
  }

  public static Command intake() {
    if (carryingNote) {
      return Commands.none();
    }
    return new ScheduleCommand(
        Commands.defer(
            () -> {
              return Commands.run(
                      () -> {
                        Pose2d intakePose = pose.get();
                        for (Pose3d note : notes) {
                          double distance =
                              Math.abs(
                                  note.getTranslation()
                                      .toTranslation2d()
                                      .getDistance(intakePose.getTranslation()));
                          if (distance > 0.2) {
                            continue;
                          }
                          carryingNote = true;
                          notes.remove(note);
                          notesPub.set(notes.toArray(new Pose3d[0]));
                          break;
                        }
                      })
                  .until(() -> carryingNote);
            },
            Set.of()));
  }

  public static Command shoot() {
    return new ScheduleCommand(
            Commands.defer(
                () -> {
                  if (!carryingNote) {
                    return Commands.none();
                  }
                  generatePath();

                  return Commands.run(
                          () -> {
                            currentNotePose = pathPosition.get(i);
                            i++;
                          })
                      .until(() -> i == pathPosition.size() - 1)
                      .finallyDo(
                          () -> {
                            i = 0;
                            carryingNote = false;
                          });
                },
                Set.of()))
        .ignoringDisable(true);
  }

  private static void generatePath() {
    double g = 9.81;
    double linearVelocity = velocity.getAsDouble() * ShooterConstants.CIRCUMFERENCE.in(Meters);

    Rotation2d shootingAngle = new Rotation2d(angle.get().getY());
    Rotation2d armPosition = shootingAngle.plus(Rotation2d.fromDegrees(180)); // flipped over origin

    Rotation2d robot = pose.get().getRotation();

    lastNotePose =
        new Pose3d(pose.get())
            .plus(
                new Transform3d(
                    new Translation3d(Inches.of(-10.465), Inches.of(0), Inches.of(25)),
                    new Rotation3d(0, armPosition.getRadians(), 0)));

    final double xVelocity = -linearVelocity * robot.getCos() * shootingAngle.getCos();
    final double yVelocity = -linearVelocity * robot.getSin() * shootingAngle.getCos();
    zVelocity = linearVelocity * shootingAngle.getSin();

    pathPosition = new ArrayList<>();
    pathPosition.add(lastNotePose);

    while (lastNotePose.getZ() > 0) {
      currentNotePose =
          new Pose3d(
              new Translation3d(
                  lastNotePose.getX() + xVelocity * PERIOD.in(Seconds),
                  lastNotePose.getY() + yVelocity * PERIOD.in(Seconds),
                  lastNotePose.getZ() + zVelocity * PERIOD.in(Seconds)),
              lastNotePose.getRotation());

      pathPosition.add(currentNotePose);
      zVelocity = zVelocity - g * PERIOD.in(Seconds);
      lastNotePose = currentNotePose;
    }
    pathPub.set(pathPosition.toArray(new Pose3d[0]));
  }
}
