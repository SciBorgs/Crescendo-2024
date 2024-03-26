package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.alliance;
import static org.sciborgs1155.robot.shooter.ShooterConstants.DEFAULT_VELOCITY;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Shooting shooting;
  private final Drive drive;
  private final Intake intake;
  private final Feeder feeder;

  public Autos(Shooting shooting, Drive drive, Intake intake, Feeder feeder) {
    chooser = new SendableChooser<>();
    this.shooting = shooting;
    this.drive = drive;
    this.intake = intake;
    this.feeder = feeder;

    chooser.setDefaultOption("Subwoofer 4 Note", followAuto("Subwoofer 4 Note"));
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

  public Command followPath(
      ChoreoTrajectory trajectory, Supplier<Optional<Rotation2d>> rotationOverride) {
    return Choreo.choreoSwerveCommand(
        trajectory,
        drive::pose,
        swerveController(
            new PIDController(Translation.P, Translation.I, Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new PIDController(Rotation.P, Rotation.I, Rotation.D),
            rotationOverride),
        s -> drive.setChassisSpeeds(s, ControlMode.CLOSED_LOOP_VELOCITY),
        () -> alliance() == Alliance.Red,
        drive);
  }

  public Command followAuto(String name) {
    Command auto = shooting.shoot(DEFAULT_VELOCITY).withTimeout(2);
    for (ChoreoTrajectory trajectory : Choreo.getTrajectoryGroup(name)) {
      auto =
          auto.andThen(
              followPath(trajectory, Optional::empty)
                  .deadlineWith(intake.intake(), feeder.forward()),
              shooting.shootWhileDriving(() -> 0, () -> 0));
    }
    return auto;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    chooser.initSendable(builder);
  }
}
