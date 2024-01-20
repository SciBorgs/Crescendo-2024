package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * @see https://github.com/wpilibsuite/allwpilib/pull/5939
 */
public class CommandRobot extends TimedRobot {

  protected CommandRobot() {
    this(kDefaultPeriod);
  }

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
}
