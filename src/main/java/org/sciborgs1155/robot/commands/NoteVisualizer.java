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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.ArrayList;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Logged;
import org.sciborgs1155.robot.pivot.PivotConstants;
import org.sciborgs1155.robot.shooter.ShooterConstants;

public class NoteVisualizer implements Logged {
  private static Pose3d[] firedNotes = new Pose3d[] {};
  private static ArrayList<Pose3d> pathPosition = new ArrayList<>();

  private static Supplier<Pose2d> pose = Pose2d::new;
  private static Supplier<Rotation2d> angle = () -> PivotConstants.STARTING_ANGLE;
  private static DoubleSupplier velocity = () -> 1;
  private static double zVelocity;

  private static Rotation2d armPosition;
  private static Pose3d currentNotePose = new Pose3d();
  private static Pose3d lastNotePose = new Pose3d();

  private static final double g = 9.81;

  private static StructArrayPublisher<Pose3d> posePub;
  private static DoublePublisher velocityPub;
  private static StructArrayPublisher<Pose3d> pathPub;

  public static void setSuppliers(
      Supplier<Pose2d> robotPose, Supplier<Rotation2d> pivotAngle, DoubleSupplier shotVelocity) {
    pose = robotPose;
    angle = pivotAngle;
    velocity = shotVelocity;
  }

  /** Set up NT publisher. Call only once before beginning to log notes. */
  public static void startPublishing() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    StructArrayTopic<Pose3d> poseTopic = inst.getStructArrayTopic("poses", Pose3d.struct);
    posePub = poseTopic.publish();

    StructArrayTopic<Pose3d> path = inst.getStructArrayTopic("path", Pose3d.struct);
    pathPub = path.publish();

    DoubleTopic velocity = inst.getDoubleTopic("y-velocity");
    velocityPub = velocity.publish();
  }

  public static void logNotes() {
    posePub.set(firedNotes);
    velocityPub.set(zVelocity);
  }

  public static Command shoot() {
    return new ScheduleCommand(
            Commands.defer(
                () -> {
                  generatePath();
                  double linearShotSpeed =
                      velocity.getAsDouble() * ShooterConstants.CIRCUMFERENCE.in(Meters);
                  armPosition = angle.get().plus(Rotation2d.fromDegrees(180));

                  // replace LENGTH with real translate to shooter
                  // Translation3d shooterTranslation =
                  //     new Translation3d(
                  //         armPosition.getCos() * LENGTH.in(Meters),
                  //         0,
                  //         armPosition.getSin() * LENGTH.in(Meters));
                  lastNotePose =
                      new Pose3d(pose.get())
                          .plus(
                              new Transform3d(
                                  OFFSET, new Rotation3d(0, armPosition.getRadians(), 0)));

                  final double xVelocity = linearShotSpeed * angle.get().getCos();
                  zVelocity = linearShotSpeed * angle.get().getSin();

                  return Commands.run(
                          () -> {
                            currentNotePose =
                                new Pose3d(
                                    new Translation3d(
                                        lastNotePose.getX() + xVelocity * PERIOD.in(Seconds),
                                        lastNotePose.getY(),
                                        lastNotePose.getZ() + zVelocity * PERIOD.in(Seconds)),
                                    new Rotation3d(0, angle.get().getRadians(), 0));
                            firedNotes = new Pose3d[] {currentNotePose};

                            zVelocity = zVelocity - g * PERIOD.in(Seconds);
                            lastNotePose = currentNotePose;
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

  public static void generatePath() {
    double linearShotSpeed = velocity.getAsDouble() * ShooterConstants.CIRCUMFERENCE.in(Meters);
    armPosition = angle.get().plus(Rotation2d.fromDegrees(180));

    // replace LENGTH with real translate to shooter
    // Translation3d shooterTranslation =
    //     new Translation3d(
    //         armPosition.getCos() * LENGTH.in(Meters), 0, armPosition.getSin() *
    // LENGTH.in(Meters));
    lastNotePose =
        new Pose3d(pose.get())
            .plus(new Transform3d(OFFSET, new Rotation3d(0, angle.get().getRadians(), 0)));

    final double xVelocity = linearShotSpeed * angle.get().getCos();
    zVelocity = linearShotSpeed * angle.get().getSin();

    pathPosition = new ArrayList<>();

    pathPosition.add(lastNotePose);
    while (lastNotePose.getZ() > 0) {
      currentNotePose =
          new Pose3d(
              new Translation3d(
                  lastNotePose.getX() + xVelocity * PERIOD.in(Seconds),
                  lastNotePose.getY(),
                  lastNotePose.getZ() + zVelocity * PERIOD.in(Seconds)),
              new Rotation3d());

      pathPosition.add(currentNotePose);
      zVelocity = zVelocity - g * PERIOD.in(Seconds);
      lastNotePose = currentNotePose;
    }

    pathPub.set(pathPosition.toArray(new Pose3d[0]));
  }
}
