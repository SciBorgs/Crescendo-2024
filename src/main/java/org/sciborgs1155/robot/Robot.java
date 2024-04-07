package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.robot.Constants.DEADBAND;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_ANGULAR_ACCEL;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_SPEED;
import static org.sciborgs1155.robot.drive.DriveConstants.TELEOP_ANGULAR_SPEED;
import static org.sciborgs1155.robot.pivot.PivotConstants.AMP_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.STARTING_ANGLE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.AMP_VELOCITY;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FakePDH;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.commands.NoteVisualizer;
import org.sciborgs1155.robot.commands.Shooting;
import org.sciborgs1155.robot.drive.Drive;
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
  private final Shooting shooting = new Shooting(shooter, pivot, feeder, drive);
  private final Alignment alignment = new Alignment(drive, pivot);

  @Log.NT
  private final SendableChooser<Command> autos =
      Autos.configureAutos(shooting, drive, pivot, shooter, intake, feeder);

  @Log.NT private double speedMultiplier = Constants.FULL_SPEED_MULTIPLIER;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
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

    SmartDashboard.putData(CommandScheduler.getInstance());
    // Log PDH
    // SmartDashboard.putData("PDH", new PowerDistribution());
    addPeriodic(() -> log("current", FakePDH.update()), PERIOD.in(Seconds));

    // Configure pose estimation updates every tick
    addPeriodic(() -> drive.updateEstimates(vision.getEstimatedGlobalPoses()), PERIOD.in(Seconds));

    // Fuck REV Robotics.!!!!
    for (var r : SparkUtils.getRunnables()) {
      addPeriodic(r, 5);
    }
    // addPeriodic(SparkUtils::update, PERIOD.in(Seconds));

    RobotController.setBrownoutVoltage(6.0);

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

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  /** Configures trigger -> command bindings */
  private void configureBindings() {
    InputStream x = InputStream.of(driver::getLeftX).negate();
    InputStream y = InputStream.of(driver::getLeftY).negate();

    InputStream r =
        InputStream.hypot(x, y)
            .log("Robot/raw joystick")
            .scale(() -> speedMultiplier)
            .clamp(1.0)
            .deadband(Constants.DEADBAND, 1.0)
            .signedPow(2.0)
            .log("Robot/processed joystick")
            .scale(MAX_SPEED.in(MetersPerSecond));

    InputStream theta = InputStream.atan(x, y);

    x = r.scale(theta.map(Math::cos)); // .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));
    y = r.scale(theta.map(Math::sin)); // .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));

    InputStream omega =
        InputStream.of(driver::getRightX)
            .negate()
            .scale(() -> speedMultiplier)
            .clamp(1.0)
            .deadband(DEADBAND, 1.0)
            .signedPow(2.0)
            .scale(TELEOP_ANGULAR_SPEED.in(RadiansPerSecond))
            .rateLimit(MAX_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)));

    drive.setDefaultCommand(drive.drive(x, y, omega));

    led.setDefaultCommand(led.setLEDTheme(LEDTheme.ALLIANCE));

    autonomous()
        .whileTrue(Commands.deferredProxy(autos::getSelected))
        .whileTrue(led.setLEDTheme(LEDTheme.RAINBOW));

    test().whileTrue(systemsCheck());

    disabled().onTrue(led.setLEDTheme(LEDTheme.NONE));

    driver.b().whileTrue(drive.zeroHeading());
    driver
        .leftBumper()
        .or(driver.rightBumper())
        .onTrue(Commands.runOnce(() -> speedMultiplier = Constants.SLOW_SPEED_MULTIPLIER))
        .onFalse(Commands.runOnce(() -> speedMultiplier = Constants.FULL_SPEED_MULTIPLIER));

    // driver shoot (x)
    driver
        .x()
        .whileTrue(shooting.shootWhileDriving(x, y))
        .whileTrue(led.setLEDTheme(LEDTheme.RAINBOW));

    // driver auto-amp (y)
    // driver
    //     .y()
    //     .whileTrue(
    //         alignment
    //             .ampAlign()
    //             // .andThen(drive.stop())
    //             .andThen(shooting.shootWithPivot(AMP_ANGLE, AMP_VELOCITY)));

    // driver climb align (a)
    driver.a().whileTrue(alignment.snapToStage(x, y));
    // 3, 9, 20 can faults
    // driver manual shooter (povUp)
    driver
        .povUp()
        .whileTrue(shooter.runShooter(-ShooterConstants.IDLE_VELOCITY.in(RadiansPerSecond)));

    // intake (right trigger / top left bump)
    operator
        .leftBumper()
        .whileTrue(intake.intake().deadlineWith(feeder.forward()))
        .whileTrue(led.setLEDTheme(LEDTheme.RAINBOW));

    // operator feed (left trigger)
    operator
        .leftTrigger()
        .whileTrue(
            shooting.shootWithPivot(PivotConstants.FEED_ANGLE, ShooterConstants.DEFAULT_VELOCITY));

    // operator climb (b)
    operator
        .b()
        .and(operator.rightTrigger())
        .whileTrue(pivot.lockedIn().deadlineWith(Commands.idle(shooter)));

    // operator note-unstuck (right bump)
    operator.rightBumper().whileTrue(pivot.runPivot(Radians.of(0.8)).alongWith(intake.backward()));

    operator.x().whileTrue(intake.backward().alongWith(feeder.backward()));

    // operator manual amp (povUp)
    operator
        .povUp()
        .whileTrue(shooting.shootWithPivot(AMP_ANGLE, AMP_VELOCITY))
        .whileTrue(led.setLEDTheme(LEDTheme.RAINBOW));

    // operator manual pivot (a)
    operator
        .a()
        .toggleOnTrue(
            pivot
                .manualPivot(
                    InputStream.of(operator::getLeftY).negate().deadband(Constants.DEADBAND, 1))
                .deadlineWith(Commands.idle(shooter)))
        .toggleOnTrue(led.setLEDTheme(LEDTheme.RAINDROP));

    // operator manual shoot (povDown)
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

  public Command systemsCheck() {
    return Commands.sequence(
            shooter.goToTest(RadiansPerSecond.of(100)),
            intake.intake().deadlineWith(feeder.forward(), shooter.runShooter(100)).withTimeout(1),
            pivot.goToTest(Radians.of(0)),
            pivot.goToTest(STARTING_ANGLE),
            drive.systemsCheck())
        .withName("Test Mechanisms");
  }

  @Override
  public void close() {
    super.close();
    led.close();
    try {
      intake.close();
      shooter.close();
      feeder.close();
      pivot.close();
      drive.close();
    } catch (Exception e) {
    }
  }
}
