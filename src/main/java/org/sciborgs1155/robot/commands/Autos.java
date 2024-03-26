package org.sciborgs1155.robot.commands;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static org.sciborgs1155.robot.Constants.alliance;
import static org.sciborgs1155.robot.shooter.ShooterConstants.DEFAULT_VELOCITY;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.drive.SwerveModule.ControlMode;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.intake.Intake;

public class Autos implements Sendable {
  private final SendableChooser<Command> chooser;
  private final EventLoop events;
  private final Shooting shooting;
  private final Drive drive;
  private final Intake intake;
  private final Feeder feeder;

  private boolean hasNote = true;
  private Optional<Rotation2d> rotation = Optional.empty();
  private final Trigger shooterReady;
  private final Trigger inRange;
  private final Trigger movingSlowly;

  public Autos(Shooting shooting, Drive drive, Intake intake, Feeder feeder) {
    chooser = new SendableChooser<>();
    events = new EventLoop();

    this.shooting = shooting;
    this.drive = drive;
    this.intake = intake;
    this.feeder = feeder;

    shooterReady = new Trigger(events, shooting::isReady);
    // canShoot = shooterReady.and(shooting::inRange);
    inRange = new Trigger(events, shooting::inRange);
    movingSlowly =
        new Trigger(
            events,
            () -> {
              var speeds = drive.getFieldRelativeChassisSpeeds();
              double magnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
              return magnitude < 2 && Math.abs(speeds.omegaRadiansPerSecond) < 2;
            });

    chooser.setDefaultOption(
        "Subwoofer 4 Note", followPath(Choreo.getTrajectory("Subwoofer 4 Note")));
  }

  public void configureBindings() {
    autonomous().and(shooterReady).and(movingSlowly).onTrue(feeder.eject().andThen(() -> hasNote = false));
    autonomous().and(inRange)
        .and(() -> hasNote)
        .whileTrue(shooting.shootWithPivot())
        .whileTrue(
            Commands.run(
                    () ->
                        rotation =
                            Optional.of(
                                Shooting.yawFromNoteVelocity(shooting.calculateNoteVelocity())))
                .finallyDo(() -> rotation = Optional.empty()));
    autonomous().and(() -> hasNote).debounce(0.4).whileFalse(intake.intake().deadlineWith(feeder.forward()));
    autonomous().and(intake.hasNote()).onTrue(Commands.runOnce(() -> hasNote = true));
  }

  public static ChoreoControlFunction swerveController(
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Supplier<Optional<Rotation2d>> rotationOverride) {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    return (pose, referenceState) -> {
      Optional<Rotation2d> override = rotationOverride.get();
      double targetRotation = override.map(Rotation2d::getRadians).orElse(referenceState.heading);

      double xFF = referenceState.velocityX;
      double yFF = referenceState.velocityY;
      double rotationFF = referenceState.angularVelocity;

      double xFeedback = xController.calculate(pose.getX(), referenceState.x);
      double yFeedback = yController.calculate(pose.getY(), referenceState.y);
      double rotationFeedback =
          rotationController.calculate(pose.getRotation().getRadians(), targetRotation);

      if (override.isPresent()) {
        rotationFF = 0;
      }

      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());
    };
  }

  public Command followPath(ChoreoTrajectory trajectory) {
    return drive
        .runOnce(
            () ->
                drive.resetOdometry(
                    (alliance() == Alliance.Red ? trajectory.flipped() : trajectory)
                        .getInitialPose()))
        .andThen(shooting.shoot(DEFAULT_VELOCITY).asProxy())
        .withTimeout(2)
        .andThen(this::configureBindings)
        .andThen(() -> hasNote = false)
        .andThen(
            Choreo.choreoSwerveCommand(
                trajectory,
                drive::pose,
                swerveController(
                    new PIDController(Translation.P, Translation.I, Translation.D),
                    new PIDController(Translation.P, Translation.I, Translation.D),
                    new PIDController(Rotation.P, Rotation.I, Rotation.D),
                    () -> rotation),
                s -> drive.setChassisSpeeds(s, ControlMode.CLOSED_LOOP_VELOCITY),
                () -> alliance() == Alliance.Red,
                drive));
  }

  // public Command followAuto(String name) {
  //   Command auto = drive.resetPose(null).shoot(DEFAULT_VELOCITY).withTimeout(2);
  //   for (ChoreoTrajectory trajectory : Choreo.getTrajectoryGroup(name)) {
  //     auto =
  //         auto.andThen(
  //             followPath(trajectory, Optional::empty)
  //                 .deadlineWith(intake.intake(), feeder.forward()),
  //             shooting.shootWhileDriving(() -> 0, () -> 0));
  //   }
  //   return auto;
  // }

  public Command get() {
    return chooser.getSelected();
  }

  public void poll() {
    events.poll();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    chooser.initSendable(builder);
  }
}
