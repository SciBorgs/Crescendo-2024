package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.alliance;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.NoteVisualizer;
import org.sciborgs1155.robot.commands.Shooting;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.drive.DriveConstants.Turn;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.intake.Intake;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.vision.Vision;

/** A command based, declarative, representation of our entire robot. */
public class Robot extends CommandRobot implements Logged {
  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // SUBSYSTEMS
  private final Drive drive = Drive.create();

  private final Intake intake =
      switch (Constants.ROBOT_TYPE) {
        case CHASSIS -> Intake.none();
        default -> Intake.create();
      };

  private final Shooter shooter =
      switch (Constants.ROBOT_TYPE) {
        case CHASSIS -> Shooter.none();
        default -> Shooter.create();
      };

  private final Feeder feeder =
      switch (Constants.ROBOT_TYPE) {
        case CHASSIS -> Feeder.none();
        default -> Feeder.create();
      };

  private final Pivot pivot =
      switch (Constants.ROBOT_TYPE) {
        case COMPLETE -> Pivot.create();
        default -> Pivot.none();
      };

  private final Vision vision = Vision.create();

  // COMMANDS
  @Log.NT private final SendableChooser<Command> autos;

  private final Shooting shooting = new Shooting(shooter, pivot, feeder, drive);

  @Log.NT private double speedMultiplier = Constants.FULL_SPEED_MULTIPLIER;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureAuto();
    autos = AutoBuilder.buildAutoChooser();
    configureGameBehavior();
    configureSubsystemDefaults();
    configureBindings();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    // Configure logging with DataLogManager, Monologue, and FailureManagement
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));
    addPeriodic(FaultLogger::update, 1);

    // Configure pose estimation updates every tick
    addPeriodic(() -> drive.updateEstimates(vision.getEstimatedGlobalPoses()), PERIOD.in(Seconds));

    if (isReal()) {
      URCL.start();
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      addPeriodic(() -> vision.simulationPeriodic(drive.getPose()), PERIOD.in(Seconds));
      NoteVisualizer.setSuppliers(
          drive::getPose, pivot::rotation, shooter::getEstimatedLaunchVelocity);
      NoteVisualizer.startPublishing();
      addPeriodic(NoteVisualizer::log, PERIOD.in(Seconds));
    }
  }

  /** Creates an input stream for a joystick. */
  private InputStream createJoystickStream(InputStream input, double maxSpeed) {
    return input
        .deadband(Constants.DEADBAND, 1)
        .negate()
        .signedPow(2)
        .scale(maxSpeed)
        .scale(() -> speedMultiplier);
  }

  public void configureAuto() {
    NamedCommands.registerCommand("Lock", drive.lock());

    AutoBuilder.configureHolonomic(
        drive::getPose,
        drive::resetOdometry,
        drive::getRobotRelativeChassisSpeeds,
        drive::driveRobotRelative,
        new HolonomicPathFollowerConfig(
            new PIDConstants(Translation.P, Translation.I, Translation.D),
            new PIDConstants(Turn.P, Turn.I, Turn.D),
            DriveConstants.MAX_SPEED.in(MetersPerSecond),
            DriveConstants.TRACK_WIDTH.divide(2).in(Meters),
            new ReplanningConfig()),
        () -> alliance() == Alliance.Red,
        drive);
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
    drive.setDefaultCommand(
        drive.drive(
            createJoystickStream(
                driver::getLeftY, // account for roborio (and navx) facing wrong direction
                DriveConstants.MAX_SPEED.in(MetersPerSecond)),
            createJoystickStream(driver::getLeftX, DriveConstants.MAX_SPEED.in(MetersPerSecond)),
            createJoystickStream(
                driver::getRightX, DriveConstants.MAX_ANGULAR_SPEED.in(RadiansPerSecond))));
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    autonomous().whileTrue(new ProxyCommand(autos::getSelected));

    driver.b().whileTrue(drive.zeroHeading());
    driver
        .leftBumper()
        .or(driver.rightBumper())
        .onTrue(Commands.runOnce(() -> speedMultiplier = Constants.SLOW_SPEED_MULTIPLIER))
        .onFalse(Commands.runOnce(() -> speedMultiplier = Constants.FULL_SPEED_MULTIPLIER));

    operator
        .a()
        .toggleOnTrue(
            pivot.manualPivot(
                InputStream.of(operator::getLeftY).negate().deadband(Constants.DEADBAND, 1)));
    operator.b().whileTrue(shooting.pivotThenShoot(PRESET_AMP_ANGLE, 70));
    operator.x().whileTrue(shooting.pivotThenShoot(Radians.of(0.5), 300));
    operator.y().whileTrue(shooting.pivotThenShoot(Radians.of(0.35), 330));

    operator
        .leftBumper()
        .and(() -> pivot.atPosition(STARTING_ANGLE.in(Radians)))
        .whileTrue(intake.intake().alongWith(feeder.forward()))
        .onFalse(feeder.retract());
    operator.rightBumper().whileTrue(feeder.forward());
    operator.povUp().whileTrue(shooter.runShooter(() -> 300));
    operator.povDown().whileTrue(shooter.runShooter(() -> 200));

    // operator.b().onTrue(pivot.runPivot(() -> )))

    // shooting into speaker when up to subwoofer
    // operator
    //     .x()
    //     .toggleOnTrue(shooting.pivotThenShoot(() -> PRESET_AMP_ANGLE.getRadians(), () -> 10));

    // operator
    //     .y()
    //     .toggleOnTrue(shooting.pivotThenShoot(() -> PRESET_SUBWOOFER_ANGLE.getRadians(), () ->
    // 11));

    // operator.a().whileTrue(intake.intake());
    // operator.x().onTrue(shooter.runShooter(() -> 100));
    // operator.y().onTrue(shooter.runShooter(() -> 0));
  }
}
