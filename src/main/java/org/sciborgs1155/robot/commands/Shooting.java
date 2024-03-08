package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.pow;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.RADIUS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.pivot.PivotConstants;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting implements Logged {

  /**
   * The conversion between shooter tangential velocity and note launch velocity. Perhaps. This may
   * also account for other errors with our model.
   */
  public static final double SIGGYS_CONSTANT = 3.7;

  // private final InterpolatingTreeMap<Double, ShootingState> shotLookup =
  //     new InterpolatingTreeMap<Double, ShootingState>(
  //         InverseInterpolator.forDouble(), ShootingState.interpolator());
  private static final InterpolatingDoubleTreeMap shotVelocityLookup =
      new InterpolatingDoubleTreeMap();

  @IgnoreLogged private final Shooter shooter;
  @IgnoreLogged private final Pivot pivot;
  @IgnoreLogged private final Feeder feeder;
  @IgnoreLogged private final Drive drive;

  public Shooting(Shooter shooter, Pivot pivot, Feeder feeder, Drive drive) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.feeder = feeder;
    this.drive = drive;

    shotVelocityLookup.put(0.0, 250.0);
    shotVelocityLookup.put(1.0, 450.0);
    shotVelocityLookup.put(4.0, 550.0);
    shotVelocityLookup.put(5.0, MAX_VELOCITY.in(RadiansPerSecond));
  }

  /**
   * Runs the shooter before feeding it the note.
   *
   * @param desiredVelocity The velocity in radians per second to shoot at.
   * @return The command to shoot at the desired velocity.
   */
  public Command shoot(Measure<Velocity<Angle>> desiredVelocity) {
    return shoot(() -> desiredVelocity.in(RadiansPerSecond), () -> true);
  }

  /**
   * Runs shooter to desired velocity, runs feeder once it reaches its velocity and shootCondition
   * is true.
   *
   * @param desiredVelocity Target velocity for the flywheel.
   * @param shootCondition Condition after which the feeder will run.
   */
  public Command shoot(DoubleSupplier desiredVelocity, BooleanSupplier shootCondition) {
    return Commands.waitUntil(() -> shooter.atSetpoint() && shootCondition.getAsBoolean())
        .andThen(feeder.eject())
        .deadlineWith(shooter.runShooter(desiredVelocity));
  }

  /**
   * Runs the pivot to an angle & runs the shooter before feeding it the note.
   *
   * @param targetAngle The desired angle of the pivot.
   * @param targetAngularVelocity The desired angular velocity of the shooter.
   * @return The command to run the pivot to its desired angle and then shoot.
   */
  public Command shootWithPivot(DoubleSupplier targetAngle, DoubleSupplier targetAngularVelocity) {
    return shoot(targetAngularVelocity, () -> pivot.atPosition(targetAngle.getAsDouble()))
        .deadlineWith(pivot.runPivot(targetAngle));
  }

  public Command shootWithPivot(
      Measure<Angle> targetAngle, Measure<Velocity<Angle>> targetVelocity) {
    return shootWithPivot(() -> targetAngle.in(Radians), () -> targetVelocity.in(RadiansPerSecond));
  }

  /** Shoots while stationary at correct flywheel speed and pivot angle, doesn't auto-turret. */
  public Command shootWithPivot() {
    return shootWithPivot(
        () -> pitchFromNoteVelocity(calculateNoteVelocity()),
        () -> rotationalVelocityFromNoteVelocity(calculateNoteVelocity()));
  }

  /**
   * Shoots while driving at a manually inputted translational velocity.
   *
   * @param vx The field relative x velocity to drive in.
   * @param vy The field relative y velocity to drive in.
   * @return A command to shote while moving.
   */
  public Command shootWhileDriving(InputStream vx, InputStream vy) {
    return shoot(
            () -> rotationalVelocityFromNoteVelocity(calculateNoteVelocity()),
            () ->
                pivot.atPosition(pitchFromNoteVelocity(calculateNoteVelocity()))
                    && drive.atHeadingSetpoint())
        .deadlineWith(
            drive.drive(
                vx.scale(0.5), vy.scale(0.5), () -> yawFromNoteVelocity(calculateNoteVelocity())),
            pivot.runPivot(() -> pitchFromNoteVelocity(calculateNoteVelocity())));
        // .unless(() -> !canShoot());
  }

  public static Pose2d robotPoseFacingSpeaker(Translation2d robotTranslation) {
    return new Pose2d(
        robotTranslation,
        translationToSpeaker(robotTranslation)
            .getAngle()
            .plus(Rotation2d.fromRadians(Math.PI / 2)));
  }

  /**
   * Calculates a vector for the desired note velocity relative to the robot for it to travel into
   * the speaker, accounting for the robot's current motion.
   *
   * @return A 3d vector representing the desired note initial velocity.
   */
  public Vector<N3> calculateNoteVelocity() {
    Pose2d robotPose = drive.pose();
    ChassisSpeeds speeds = drive.getFieldRelativeChassisSpeeds();
    // robot velocity is negated because our shooter is on the opposite end of our robot
    Vector<N3> robotVelocity =
        VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0).times(-1);
    Translation2d difference = translationToSpeaker(robotPose.getTranslation());
    double shotVelocity = calculateStationaryVelocity(difference.getNorm());
    Rotation3d noteOrientation =
        new Rotation3d(
            0,
            -calculateStationaryPitch(
                robotPoseFacingSpeaker(robotPose.getTranslation()), shotVelocity, pivot.position()),
            difference.getAngle().getRadians());
    // rotate unit forward vector by note orientation and scale by our shot velocity
    Vector<N3> noteVelocity =
        new Translation3d(1, 0, 0).rotateBy(noteOrientation).toVector().unit().times(shotVelocity);

    return noteVelocity.minus(robotVelocity);
  }

  /**
   * Returns the pose of the shooter.
   *
   * @return The pose of the shooter in 3d space.
   */
  @Log.NT
  public Pose3d shooterPose() {
    return new Pose3d(drive.pose())
        .transformBy(pivot.transform())
        .transformBy(PivotConstants.SHOOTER_FROM_AXLE);
  }

  public static Pose3d shooterPose(Transform3d pivot, Pose2d robot) {
    return new Pose3d(robot).transformBy(pivot).transformBy(PivotConstants.SHOOTER_FROM_AXLE);
  }

  /**
   * Calculates if the robot can make its current shot.
   *
   * @return Whether the robot can shoot from its current position at its current velocity.
   */
  @Log.NT
  public boolean canShoot() {
    Vector<N3> shot = calculateNoteVelocity();
    double pitch = pitchFromNoteVelocity(shot);
    return MIN_ANGLE.in(Radians) < pitch
        && pitch < MAX_ANGLE.in(Radians)
        && Math.abs(rotationalVelocityFromNoteVelocity(shot)) < MAX_VELOCITY.in(RadiansPerSecond);
  }

  /**
   * Calculates pitch from note initial velocity vector. If given a robot relative initial velocity
   * vector, the return value will also be the pivot angle.
   *
   * @param velocity Note initial velocity vector
   * @return Pitch/pivot angle
   */
  public static double pitchFromNoteVelocity(Vector<N3> velocity) {
    return Math.atan(velocity.get(2) / VecBuilder.fill(velocity.get(0), velocity.get(1)).norm());
  }

  /**
   * Calculates heading from note initial velocity vector. If given a robot relative initial
   * velocity vector, the return value will be the target robot heading.
   *
   * @param velocity Note initial velocity vector
   * @return Heading
   */
  public static Rotation2d yawFromNoteVelocity(Vector<N3> velocity) {
    return Rotation2d.fromRadians(Math.PI).plus(new Rotation2d(velocity.get(0), velocity.get(1)));
  }

  /**
   * Calculates magnitude of initial velocity vector of note, in radians per second. If given a
   * robot relative initial velocity vector, the return value will be the target flywheel speed
   * (ish).
   *
   * @param velocity Note initial velocity vector relative to the robot
   * @return Flywheel speed (rads / s)
   */
  public static double rotationalVelocityFromNoteVelocity(Vector<N3> velocity) {
    return velocity.norm() / RADIUS.in(Meters) * SIGGYS_CONSTANT;
  }

  /**
   * Converts between flywheel speed and note speed
   *
   * @param flywheelSpeed Flywheel speed in radians per second
   * @return Note speed in meters per second
   */
  public static double flywheelToNoteSpeed(double flywheelSpeed) {
    return flywheelSpeed * RADIUS.in(Meters) / SIGGYS_CONSTANT;
  }

  public static Translation2d translationToSpeaker(Translation2d robotTranslation) {
    return speaker().toTranslation2d().minus(robotTranslation);
  }

  public static double calculateStationaryVelocity(double distance) {
    return flywheelToNoteSpeed(shotVelocityLookup.get(distance));
  }

  /**
   * Calculates a stationary pitch from a pose so that the note goes into the speaker.
   *
   * @param shooterPose The pose of the shooter.
   * @param velocity The magnitude of velocity to launch the note at.
   * @return The pitch to shoot the note at.
   */
  public static double calculateStationaryPitch(
      Pose2d robotPose, double velocity, double prevPitch) {
    return calculateStationaryPitch(robotPose, velocity, prevPitch, 0);
  }

  private static double calculateStationaryPitch(
      Pose2d robotPose, double velocity, double prevPitch, int i) {
    double G = 9.81;
    Translation3d shooterTranslation =
        shooterPose(Pivot.transform(-prevPitch), robotPose).getTranslation();
    double dist = translationToSpeaker(shooterTranslation.toTranslation2d()).getNorm();
    double h = speaker().getZ() - shooterTranslation.getZ();
    double denom = (G * pow(dist, 2));
    double rad =
        pow(dist, 2) * pow(velocity, 4)
            - G * pow(dist, 2) * (G * pow(dist, 2) + 2 * h * pow(velocity, 2));
    double pitch = Math.atan((1 / (denom)) * (dist * pow(velocity, 2) - Math.sqrt(rad)));
    if (Math.abs(pitch - prevPitch) < 0.005 || i > 50) {
      return pitch;
    }
    return calculateStationaryPitch(robotPose, velocity, pitch, i + 1);
  }
}
