package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
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
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.led.Leds;
import org.sciborgs1155.robot.led.Leds.LEDTheme;

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
  private final Drive drive = Drive.create();
  private final Leds led = new Leds();

  // COMMANDS
  @Log.NT private final Autos autos = new Autos();

  @Log.NT private double speedMultiplier = Constants.FULL_SPEED;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    configureGameBehavior();
    configureSubsystemDefaults();
    configureBindings();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    // Configure logging with DataLogManager, Monologue, FailureManagement, and URCL
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, kDefaultPeriod);
    FaultLogger.setupLogging();
    addPeriodic(FaultLogger::update, 1);

    if (isReal()) {
      URCL.start();
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /** Creates an input stream for a joystick. */
  private InputStream createJoystickStream(InputStream input, double maxSpeed, double maxRate) {
    return input
        .deadband(Constants.DEADBAND, 1)
        .negate()
        .scale(maxSpeed)
        .scale(() -> speedMultiplier)
        .signedPow(2)
        .rateLimit(maxRate);
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
    drive.setDefaultCommand(
        drive.drive(
            createJoystickStream(
                driver::getLeftX,
                DriveConstants.MAX_SPEED.in(MetersPerSecond),
                DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)),
            createJoystickStream(
                driver::getLeftY,
                DriveConstants.MAX_SPEED.in(MetersPerSecond),
                DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)),
            createJoystickStream(
                driver::getRightX,
                DriveConstants.MAX_ANGULAR_SPEED.in(RadiansPerSecond),
                DriveConstants.MAX_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)))));
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    autonomous().whileTrue(new ProxyCommand(autos::get));
    FaultLogger.onFailing(f -> Commands.print(f.toString()));

    driver
        .leftBumper()
        .or(driver.rightBumper())
        .onTrue(Commands.runOnce(() -> speedMultiplier = Constants.FULL_SPEED))
        .onFalse(Commands.run(() -> speedMultiplier = Constants.SLOW_SPEED));

    /*
      currently configured to keyboard 1 in joystick[0], controls are m,./
      look in Leds.java and LedConstants.java for all themes and what they look like
      (or just allow it to print the Led data and read the hex values)

      you can also use led.setTheme(LEDTheme.BXSCIFLASH); to set a premade theme
      you can also use led.setColor(Color.kOrange); to set a single color

      note: unless a strategy is to confuse opponents with flashing lights to hypnotise them, this
      should probably not be binded to a XBOX controller but instead used in other systems (ex.
      bagel/donut/note location)
    */
    operator.a().onTrue(led.setTheme(LEDTheme.LIT));
    operator.b().onTrue(led.setTheme(LEDTheme.CHASE));
    operator.x().onTrue(led.setColor(Color.kLime));
    operator.y().onTrue(led.setTheme(LEDTheme.RAINDROP));
  }

  @Override
  public void close() {
    led.close();
    super.close();
  }
}
