package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
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
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.intake.Intake;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.vision.Vision;
import org.sciborgs1155.robot.vision.VisionConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {
  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // SUBSYSTEMS
  // private final Vision vision =
  //     new Vision(VisionConstants.FRONT_CAMERA_CONFIG, VisionConstants.SIDE_CAMERA_CONFIG);
  private final Vision vision =
      new Vision(VisionConstants.FRONT_CAMERA_CONFIG, VisionConstants.SIDE_CAMERA_CONFIG);

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

  // COMMANDS
  @Log.NT private final SendableChooser<Command> autos;

  private final Shooting shooting = new Shooting(shooter, pivot, feeder);

  @Log.NT private double speedMultiplier = Constants.FULL_SPEED_MULTIPLIER;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    drive.configureAuto();
    registerCommands();
    autos = AutoBuilder.buildAutoChooser();
    configureGameBehavior();
    configureSubsystemDefaults();
    configureBindings();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    for (var subsystem : List.of(drive, pivot, intake, shooter)) {
      SmartDashboard.putData((Sendable) subsystem);
    }
    SmartDashboard.putData(CommandScheduler.getInstance());
    // Configure logging with DataLogManager, Monologue, and FailureManagement
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, kDefaultPeriod);
    FaultLogger.setupLogging();
    addPeriodic(FaultLogger::update, 1);

    // Configure pose estimation updates every half-tick
    // addPeriodic(
    //     () -> drive.updateEstimates(vision.getEstimatedGlobalPoses()), kDefaultPeriod / 2.0);

    if (isReal()) {
      URCL.start();
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      addPeriodic(() -> vision.simulationPeriodic(drive.getPose()), kDefaultPeriod);
      NoteVisualizer.setSuppliers(drive::getPose, pivot::rotation, shooter::getVelocity);
      NoteVisualizer.startPublishing();
      addPeriodic(NoteVisualizer::log, kDefaultPeriod);
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

  /** Registers all named commands, which will be used by pathplanner */
  private void registerCommands() {
    NamedCommands.registerCommand("lock", drive.lock());
    NamedCommands.registerCommand(
        "shoot", shooting.pivotThenShoot(() -> PRESET_PODIUM_ANGLE.getRadians(), () -> 11).withTimeout(3));
    NamedCommands.registerCommand("intake", intake.intake().withTimeout(1.6));
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    autonomous().whileTrue(new ProxyCommand(autos::getSelected));

    driver.b().whileTrue(drive.zeroHeading());
    driver
        .x()
        .toggleOnTrue(
            drive
                .driveFacingTarget(
                    createJoystickStream(
                        driver::getLeftY, // account for roborio (and navx) facing wrong direction
                        DriveConstants.MAX_SPEED.in(MetersPerSecond)),
                    createJoystickStream(
                        driver::getLeftX, DriveConstants.MAX_SPEED.in(MetersPerSecond)),
                    Translation2d::new)
                .until(
                    () ->
                        Math.abs(Math.hypot(driver.getRightX(), driver.getRightY()))
                            > Constants.DEADBAND));

    driver
        .leftBumper()
        .or(driver.rightBumper())
        .onTrue(Commands.runOnce(() -> speedMultiplier = Constants.SLOW_SPEED_MULTIPLIER))
        .onFalse(Commands.runOnce(() -> speedMultiplier = Constants.FULL_SPEED_MULTIPLIER));

    // shooting
    operator
        .x()
        .toggleOnTrue(
            shooter.runShooter(() -> 12));
    // pivot into speaker when up to podium.
    operator
        .y()
        .toggleOnTrue(
            pivot.runPivot(() -> PRESET_PODIUM_ANGLE.getRadians()));
    // pivot into amp when up to amp.
    operator
        .a()
        .toggleOnTrue(
            pivot.runPivot(() -> PRESET_AMP_ANGLE.getRadians()));
    // moving pivot to starting config.
    operator.b().toggleOnTrue(pivot.runPivot(() -> STARTING_ANGLE.getRadians()));
    // outtake
    operator.leftBumper().toggleOnTrue(intake.outtake());
    // intake
    operator.rightBumper().toggleOnTrue(intake.intake());
    // manually control the pivot(climb)
    operator.povUp().toggleOnTrue(pivot.manualPivot(operator::getLeftY));

    intake.hasNote().onTrue(rumble(RumbleType.kBothRumble, 0.5));

    intake
        .hasNote()
        .onTrue(
            pivot
                .runPivot(() -> STARTING_ANGLE.getRadians())
                .andThen(
                    feeder
                        .runFeeder(1)
                        // .until(feeder.topBB()))
                        .withTimeout(3)));
  }

  public Command rumble(RumbleType RumbleType, double strength) {
    return Commands.run(() -> operator.getHID().setRumble(RumbleType, strength))
        .alongWith(Commands.run(() -> driver.getHID().setRumble(RumbleType, strength)));
  }
}
