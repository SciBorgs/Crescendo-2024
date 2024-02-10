package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.PRESET_SUBWOOFER_ANGLE;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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
import org.sciborgs1155.robot.commands.Shooting;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.feeder.Feeder;
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
  private final Vision vision =
      new Vision(VisionConstants.FRONT_CAMERA_CONFIG, VisionConstants.SIDE_CAMERA_CONFIG);

  @Log.NT private final Drive drive = Drive.create();

  @Log.NT
  private final Shooter shooter =
      switch (Constants.ROBOT_TYPE) {
        case CHASSIS -> Shooter.createNone();
        default -> Shooter.create();
      };

  @Log.NT
  private final Feeder feeder =
      switch (Constants.ROBOT_TYPE) {
        case CHASSIS -> Feeder.createNone();
        default -> Feeder.create();
      };

  @Log.NT
  private final Pivot pivot =
      switch (Constants.ROBOT_TYPE) {
        case COMPLETE -> Pivot.create();
        default -> Pivot.createNone();
      };

  // COMMANDS
  @Log.NT private final SendableChooser<Command> autos;

  private final Shooting shooting = new Shooting(shooter, pivot, feeder);

  @Log.NT private double speedMultiplier = Constants.FULL_SPEED_MULTIPLIER;

  // @Log.NT private double simPosition = 0;

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
    // Configure logging with DataLogManager, Monologue, and FailureManagement
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, kDefaultPeriod);
    FaultLogger.setupLogging();
    addPeriodic(FaultLogger::update, 1);

    // Configure pose estimation updates every half-tick
    addPeriodic(
        () -> drive.updateEstimates(vision.getEstimatedGlobalPoses()), kDefaultPeriod / 2.0);

    if (isReal()) {
      URCL.start();
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      addPeriodic(() -> vision.simulationPeriodic(drive.getPose()), kDefaultPeriod);
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
    NamedCommands.registerCommand("Lock", drive.lock());
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    autonomous().whileTrue(new ProxyCommand(autos::getSelected));
    // FaultLogger.onFailing(
    //     f ->
    //         drive
    //             .lock()
    //             .alongWith(
    //                 Commands.run(
    //                         () ->
    //                             DriverStation.reportError(
    //                                 "pain and suffering and " + f.toString(), false))
    //                     .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));
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

    operator.a().toggleOnTrue(pivot.manualPivot(operator::getLeftY));

    // shooting into speaker when up to subwoofer
    operator.x().toggleOnTrue(shooting.pivotThenShoot(() -> PRESET_SUBWOOFER_ANGLE, () -> 2));
  }
}
