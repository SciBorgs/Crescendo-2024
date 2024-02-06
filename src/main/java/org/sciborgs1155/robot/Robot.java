package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Logged;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.led.LedStrip;
import org.sciborgs1155.robot.led.LedStrip.LEDTheme;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {

  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  // private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // SUBSYSTEMS
  // @Log.NT private final Drive drive = Drive.create();

  // private final Flywheel flywheel = Flywheel.create();
  // private final Feeder feeder = Feeder.create();
  // private final Pivot pivot = Pivot.create();
  private final LedStrip led = new LedStrip();

  // COMMANDS
  // @Log.NT private final SendableChooser<Command> autos = AutoBuilder.buildAutoChooser();

  // @Log.NT private final Shooting shooting = new Shooting(flywheel, pivot, feeder);

  // @Log.NT private double speedMultiplier = Constants.FULL_SPEED_MULTIPLIER;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    registerCommands();
    configureGameBehavior();
    configureSubsystemDefaults();
    configureBindings();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    // // Configure logging with DataLogManager, Monologue, FailureManagement, and URCL
    // DataLogManager.start();
    // Monologue.setupMonologue(this, "/Robot", false, true);
    // addPeriodic(Monologue::updateAll, kDefaultPeriod);
    // FaultLogger.setupLogging();
    // addPeriodic(FaultLogger::update, 1);

    // if (isReal()) {
    //   URCL.start();
    // } else {
    //   DriverStation.silenceJoystickConnectionWarning(true);
    // }
  }

  /** Creates an input stream for a joystick. */
  // private InputStream createJoystickStream(InputStream input, double maxSpeed, double maxRate) {
  //   return input
  //       .deadband(Constants.DEADBAND, 1)
  //       .negate()
  //       .scale(maxSpeed)
  //       .scale(() -> speedMultiplier)
  //       .signedPow(2)
  //       .rateLimit(maxRate);
  // }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
    // drive.setDefaultCommand(
    //     drive.drive(
    //         createJoystickStream(
    //             driver::getLeftX,
    //             DriveConstants.MAX_SPEED.in(MetersPerSecond),
    //             DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)),
    //         createJoystickStream(
    //             driver::getLeftY,
    //             DriveConstants.MAX_SPEED.in(MetersPerSecond),
    //             DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)),
    //         createJoystickStream(
    //             driver::getRightX,
    //             DriveConstants.MAX_ANGULAR_SPEED.in(RadiansPerSecond),
    //             DriveConstants.MAX_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)))));
  }

  /** Registers all named commands, which will be used by pathplanner */
  private void registerCommands() {
    // EX: NamedCommands.registerCommand(name, command);
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    // autonomous().whileTrue(new ProxyCommand(autos::getSelected));
    // FaultLogger.onFailing(f -> Commands.print(f.toString()));

    // driver
    //     .leftBumper()
    //     .or(driver.rightBumper())
    //     .onTrue(Commands.runOnce(() -> speedMultiplier = Constants.FULL_SPEED_MULTIPLIER))
    //     .onFalse(Commands.run(() -> speedMultiplier = Constants.SLOW_SPEED_MULTIPLIER));

    // operator.a().toggleOnTrue(pivot.manualPivot(operator::getLeftY));

    // shooting
    //     .canShoot()
    //     .onTrue(led.setLEDTheme(LEDTheme.CANSHOOT))
    //     .onFalse(led.setLEDTheme(LEDTheme.FIRE));

    // // shooting into speaker when up to subwoofer
    // operator
    //     .x()
    //     .and(shooting.canShoot())
    //     .toggleOnTrue(
    //         shooting.pivotThenShoot(
    //             () -> new Rotation2d(PRESET_SUBWOOFER_ANGLE),
    //             () -> PRESET_SUBWOOFER_VELOCITY.in(RadiansPerSecond)));

    // // assuming x is shoot button, will rumble if cant shoot
    // operator
    //     .x()
    //     .and(() -> shooting.canShoot().getAsBoolean() == false)
    //     .toggleOnTrue(rumble(RumbleType.kBothRumble, 0.5));

    operator.a().toggleOnTrue(led.setLEDTheme(LEDTheme.RAINBOW));
  }

  @Override
  public void close() {
    led.close();
    super.close();
  }

  // public Command rumble(RumbleType rumbleType, double strength) {
  //   return Commands.run(() -> operator.getHID().setRumble(rumbleType, strength));
  // }
}
