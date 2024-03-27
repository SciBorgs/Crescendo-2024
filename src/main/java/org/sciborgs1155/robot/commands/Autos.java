package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.alliance;
import static org.sciborgs1155.robot.shooter.ShooterConstants.DEFAULT_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.IDLE_VELOCITY;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Optional;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.drive.SwerveModule.ControlMode;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.intake.Intake;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.shooter.Shooter;

public class Autos {
  private static Optional<Rotation2d> rotation = Optional.empty();

  public static SendableChooser<Command> configureAutos(
      Shooting shooting, Drive drive, Pivot pivot, Shooter shooter, Intake intake, Feeder feeder) {
    AutoBuilder.configureHolonomic(
        drive::pose,
        drive::resetOdometry,
        drive::getRobotRelativeChassisSpeeds,
        s -> drive.setChassisSpeeds(s, ControlMode.CLOSED_LOOP_VELOCITY),
        new HolonomicPathFollowerConfig(
            new PIDConstants(Translation.P, Translation.I, Translation.D),
            new PIDConstants(Rotation.P, Rotation.I, Rotation.D),
            DriveConstants.MAX_SPEED.in(MetersPerSecond),
            DriveConstants.RADIUS.in(Meters),
            new ReplanningConfig()),
        () -> alliance() == Alliance.Red,
        drive);

    PPHolonomicDriveController.setRotationTargetOverride(() -> rotation);

    NamedCommands.registerCommand(
        "intake", new ScheduleCommand(intake.intake().deadlineWith(feeder.forward())));
    NamedCommands.registerCommand(
        "aim",
        // shooting
        //     .aimWithoutShooting()
        //     .alongWith(
        shooter
            .runShooter(DEFAULT_VELOCITY.in(RadiansPerSecond))
            .alongWith(
                Commands.run(
                    () ->
                        rotation =
                            Optional.of(
                                Shooting.yawFromNoteVelocity(shooting.calculateNoteVelocity()))))
            .finallyDo(() -> rotation = Optional.empty()));
    NamedCommands.registerCommand(
        "shoot", shooting.shootWhileDriving(() -> 0, () -> 0).withTimeout(2));
    // .deadlineWith(drive.lock()));
    NamedCommands.registerCommand(
        "shoot-after-intake",
        Commands.waitSeconds(0.3)
            .andThen(shooting.shootWithPivot())
            .withTimeout(2.5)
            .deadlineWith(
                drive.drive(
                    () -> 0,
                    () -> 0,
                    () -> Shooting.yawFromNoteVelocity(shooting.calculateNoteVelocity()))));
    NamedCommands.registerCommand(
        "shoot-subwoofer", shooting.shoot(IDLE_VELOCITY).withTimeout(2.2));

    FollowPathCommand.warmupCommand().schedule();
    return AutoBuilder.buildAutoChooser("Subwoofer 5 Note");
  }
}
