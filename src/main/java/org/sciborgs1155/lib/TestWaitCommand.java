package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.Function;

  public class TestWaitCommand extends Command {
    private double elapsed = 0;
    private final double duration;
    private static final double TICK_RATE = 0.02;

    public TestWaitCommand(double seconds) {
      this.duration = seconds;
    }

    @Override
    public void execute() {
      elapsed += TICK_RATE;
    }

    @Override
    public boolean isFinished() {
      return elapsed >= duration;
    }

    public boolean runsWhenDisabled() {
      return true;
    }

    public static ParallelRaceGroup withTimeout(Command command, boolean unitTest, double seconds) {
      return command.raceWith(unitTest ? new TestWaitCommand(seconds) : new WaitCommand(seconds));
    }

    public static Function<Boolean, Command> withTimeout(Command command, double seconds) {
      return u -> withTimeout(command, u, seconds);
    }
  }

