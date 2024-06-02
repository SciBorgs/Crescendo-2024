package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.Constants.Field.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
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
import java.util.Queue;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class NoteVisualizer implements Logged {
  // notes
  private static Queue<Pose3d> notes =
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
  private static boolean carryingNote = true;
  private static List<Pose3d> pathPosition = new ArrayList<>();

  // suppliers
  private static Supplier<Pose2d> drive = Pose2d::new;
  private static Supplier<Pose3d> shooter = Pose3d::new;
  private static Supplier<ChassisSpeeds> speeds = ChassisSpeeds::new;
  private static DoubleSupplier velocity = () -> 0;

  private static final Vector<N3> GRAVITY = VecBuilder.fill(0, 0, -9.81);

  private static int step = 0;

  private static StructArrayPublisher<Pose3d> notesPub;

  // publishers
  private static NetworkTable table;
  private static StructArrayPublisher<Pose3d> notePathPub;
  private static StructPublisher<Pose3d> shotNotePub;

  public static void setSuppliers(
      Supplier<Pose2d> drivePose,
      Supplier<Pose3d> pivotPose,
      Supplier<ChassisSpeeds> driveSpeeds,
      DoubleSupplier shotVelocity) {
    drive = drivePose;
    shooter = pivotPose;
    speeds = driveSpeeds;
    velocity = shotVelocity;
  }

  /** Set up NT publisher. Call only once before beginning to log notes. */
  public static void startPublishing() {
    table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("Notes");

    StructArrayTopic<Pose3d> notesTopic = table.getStructArrayTopic("notes", Pose3d.struct);
    StructArrayTopic<Pose3d> notePathTopic = table.getStructArrayTopic("note path", Pose3d.struct);
    StructTopic<Pose3d> shotNoteTopic = table.getStructTopic("shot note", Pose3d.struct);

    notesPub = notesTopic.publish();
    notePathPub = notePathTopic.publish();
    shotNotePub = shotNoteTopic.publish();

    notesPub.set(notes.toArray(new Pose3d[0]));
  }

  public static Command intake() {
    return new ScheduleCommand(
            Commands.defer(
                () -> {
                  if (carryingNote) {
                    return Commands.none();
                  }
                  return Commands.run(
                      () -> {
                        Pose2d intakePose = drive.get();
                        for (Pose3d note : notes) {
                          double distance =
                              Math.abs(
                                  note.getTranslation()
                                      .toTranslation2d()
                                      .getDistance(intakePose.getTranslation()));
                          if (distance > 0.6) {
                            continue;
                          }
                          carryingNote = true;
                          notes.remove(note);
                          notesPub.set(notes.toArray(new Pose3d[0]));
                          break;
                        }
                      });
                },
                Set.of()))
        .unless(Robot::isReal);
  }

  public static Command shoot() {
    return new ScheduleCommand(
            Commands.defer(
                () -> {
                  var poses = generatePath();
                  if (poses.length == 0) return Commands.runOnce(() -> {});
                  step = 0;
                  notePathPub.set(poses);
                  return Commands.run(
                          () -> {
                            shotNotePub.set(poses[step]);
                            step++;
                          })
                      .until(() -> step >= poses.length - 1)
                      .andThen(Commands.waitSeconds(1))
                      .finallyDo(
                          () -> {
                            shotNotePub.set(new Pose3d());
                            notePathPub.set(new Pose3d[0]);
                            carryingNote = false;
                          });
                },
                Set.of()))
        // .unless(() -> !carryingNote)
        .unless(Robot::isReal)
        .ignoringDisable(true);
  }

  public static boolean hasNote() {
    return carryingNote;
  }

  private static Pose3d[] generatePath() {
    double shotVelocity = velocity.getAsDouble();
    ChassisSpeeds driveSpeeds = speeds.get();

    Pose3d pose = shooter.get();
    Vector<N3> position = pose.getTranslation().toVector();

    Vector<N3> driveVelocity =
        VecBuilder.fill(driveSpeeds.vxMetersPerSecond, driveSpeeds.vyMetersPerSecond, 0);

    Vector<N3> velocity =
        new Translation3d(1, 0, 0)
            .rotateBy(pose.getRotation())
            .toVector()
            .unit()
            .times(shotVelocity)
            .minus(driveVelocity)
            .times(-1);

    pathPosition = new ArrayList<>();

    while (inField(pose) && pose.getZ() > 0) {
      pathPosition.add(pose);

      pose = new Pose3d(new Translation3d(position), pose.getRotation());

      position = position.plus(velocity.times(PERIOD.in(Seconds)));
      velocity = velocity.plus(GRAVITY.times(PERIOD.in(Seconds)));
    }
    // carryingNote = false;
    // notePathPub.set(pathPosition.toArray(new Pose3d[0]));
    return pathPosition.toArray(Pose3d[]::new);
  }
}
