package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.alliance;
import static org.sciborgs1155.robot.pivot.PivotConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.DEFAULT_VELOCITY;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Climbing;
import org.sciborgs1155.robot.commands.NoteVisualizer;
import org.sciborgs1155.robot.commands.Shooting;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.drive.SwerveModule.ControlMode;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.intake.Intake;
import org.sciborgs1155.robot.led.LedStrip;
import org.sciborgs1155.robot.led.LedStrip.LEDTheme;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.pivot.PivotConstants;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.shooter.ShooterConstants;
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

  private final LedStrip led = new LedStrip();

  private final Vision vision = Vision.create();

  // COMMANDS
  @Log.NT private final SendableChooser<Command> autos;

  private final Shooting shooting = new Shooting(shooter, pivot, feeder, drive);
  private final Climbing climbing = new Climbing(drive, pivot);

  @Log.NT private double speedMultiplier = Constants.FULL_SPEED_MULTIPLIER;

  @Log.NT public static double current;

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
    addPeriodic(FaultLogger::update, 2);
    addPeriodic(
        () -> log("dist", Shooting.translationToSpeaker(drive.pose().getTranslation()).getNorm()),
        kDefaultPeriod);

    // Log PDH
    // SmartDashboard.putData("PDH", new PowerDistribution());
    addPeriodic(
        () ->
            current = FaultLogger.sparks.stream().mapToDouble(CANSparkBase::getOutputCurrent).sum(),
        kDefaultPeriod);

    // Configure pose estimation updates every tick
    addPeriodic(() -> drive.updateEstimates(vision.getEstimatedGlobalPoses()), PERIOD.in(Seconds));

    if (isReal()) {
      URCL.start();
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      addPeriodic(() -> vision.simulationPeriodic(drive.pose()), PERIOD.in(Seconds));
      NoteVisualizer.setSuppliers(
          drive::pose,
          shooting::shooterPose,
          drive::getFieldRelativeChassisSpeeds,
          shooter::tangentialVelocity);
      NoteVisualizer.startPublishing();
    }
  }

  /** Creates an input stream for a joystick. */
  private InputStream createJoystickStream(InputStream input, double maxSpeed) {
    return input
        .deadband(Constants.DEADBAND, 1)
        .negate()
        .signedPow(2)
        .scale(maxSpeed)
        .scale(() -> speedMultiplier)
        .rateLimit(DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond));
  }

  public void configureAuto() {
    // register named commands for auto
    NamedCommands.registerCommand(
        "shoot", shooting.shootWhileDriving(() -> 0, () -> 0).withTimeout(2));
    NamedCommands.registerCommand(
        "intake",
        intake
            .intake()
            .deadlineWith(feeder.forward())
            .andThen(
                intake
                    .stop()
                    .alongWith(feeder.runFeeder(0))
                    .alongWith(Commands.waitSeconds(0.5).andThen(shooting.aimWithoutShooting()))));
    NamedCommands.registerCommand(
        "subwoofer-shoot", shooting.shoot(DEFAULT_VELOCITY).withTimeout(3));
    // NamedCommands.registerCommand("stop", drive.driveRobotRelative);

    // configure auto
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
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
    drive.setDefaultCommand(
        drive.drive(
            createJoystickStream(driver::getLeftY, DriveConstants.MAX_SPEED.in(MetersPerSecond)),
            createJoystickStream(driver::getLeftX, DriveConstants.MAX_SPEED.in(MetersPerSecond)),
            createJoystickStream(
                driver::getRightX, DriveConstants.TELEOP_ANGULAR_SPEED.in(RadiansPerSecond))));
    led.setDefaultCommand(led.setLEDTheme(LEDTheme.BLUE));
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    autonomous()
        .whileTrue(Commands.deferredProxy(autos::getSelected))
        .whileTrue(led.setLEDTheme(LEDTheme.RAINBOW));

    driver.b().whileTrue(drive.zeroHeading());
    driver
        .leftBumper()
        .or(driver.rightBumper())
        .onTrue(Commands.runOnce(() -> speedMultiplier = Constants.SLOW_SPEED_MULTIPLIER))
        .onFalse(Commands.runOnce(() -> speedMultiplier = Constants.FULL_SPEED_MULTIPLIER));

    driver
        .a()
        .whileTrue(
            climbing
                .snapToStage(
                    createJoystickStream(
                        driver::getLeftY, DriveConstants.MAX_SPEED.in(MetersPerSecond)),
                    createJoystickStream(
                        driver::getLeftX, DriveConstants.MAX_SPEED.in(MetersPerSecond)))
                .alongWith(
                    climbing.angleClimber())); // stop holding the button in order to climb with the
    // pivot manually

    operator
        .a()
        .toggleOnTrue(
            pivot
                .manualPivot(
                    InputStream.of(operator::getLeftY).negate().deadband(Constants.DEADBAND, 1))
                .deadlineWith(Commands.idle(shooter)))
        .toggleOnTrue(led.setLEDTheme(LEDTheme.RAINDROP));

    operator
        .b()
        .and(operator.rightTrigger())
        .whileTrue(pivot.lockedIn().deadlineWith(Commands.idle(shooter)));

    driver
        .x()
        .whileTrue(
            shooting.shootWhileDriving(
                createJoystickStream(
                    driver::getLeftY, DriveConstants.MAX_SPEED.in(MetersPerSecond)),
                createJoystickStream(
                    driver::getLeftX, DriveConstants.MAX_SPEED.in(MetersPerSecond))))
        .whileTrue(led.setLEDTheme(LEDTheme.RAINBOW));

    driver
        .y()
        .or(operator.povUp())
        .whileTrue(
            shooting.shootWithPivot(PivotConstants.PRESET_AMP_ANGLE, ShooterConstants.AMP_VELOCITY))
        .whileTrue(led.setLEDTheme(LEDTheme.RAINBOW));

    driver
        .rightTrigger()
        .or(operator.leftBumper())
        .and(() -> pivot.atPosition(MAX_ANGLE.in(Radians)))
        .whileTrue(intake.intake().deadlineWith(feeder.forward()))
        .whileTrue(led.setLEDTheme(LEDTheme.RAINBOW));

    driver
        .povUp()
        .whileTrue(shooter.runShooter(-ShooterConstants.IDLE_VELOCITY.in(RadiansPerSecond)));

    operator.rightBumper().whileTrue(intake.backward());
    operator
        .povDown()
        .whileTrue(shooting.shoot(RadiansPerSecond.of(350)))
        .whileTrue(led.setLEDTheme(LEDTheme.RAINBOW));

    intake
        .hasNote()
        .onTrue(rumble(RumbleType.kLeftRumble, 0.3))
        .whileTrue(led.setLEDTheme(LEDTheme.ORANGE));
    feeder
        .noteAtShooter()
        .onFalse(rumble(RumbleType.kRightRumble, 0.3))
        .whileTrue(led.setLEDTheme(LEDTheme.ORANGE));
  }

  public Command rumble(RumbleType rumbleType, double strength) {
    return Commands.runOnce(
            () -> {
              driver.getHID().setRumble(rumbleType, strength);
              operator.getHID().setRumble(rumbleType, strength);
            })
        .andThen(Commands.waitSeconds(0.3))
        .finallyDo(
            () -> {
              driver.getHID().setRumble(rumbleType, 0);
              operator.getHID().setRumble(rumbleType, 0);
            });
  }
}
