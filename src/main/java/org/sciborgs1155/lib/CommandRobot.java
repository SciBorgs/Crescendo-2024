package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is a jank workaround, a full (risky) solution would be replacing {@link
 * edu.wpi.first.wpilibj.IterativeRobotBase}
 */
public class CommandRobot extends TimedRobot {

  protected CommandRobot(double period) {
    super(period);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationPeriodic() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  public Trigger robot() {
    return new Trigger(() -> true);
  }

  public Trigger autonomous() {
    return new Trigger(DriverStation::isAutonomous);
  }

  public Trigger teleop() {
    return new Trigger(DriverStation::isTeleop);
  }

  public Trigger test() {
    return new Trigger(DriverStation::isTest);
  }

  public Trigger simulation() {
    return new Trigger(TimedRobot::isSimulation);
  }
}
