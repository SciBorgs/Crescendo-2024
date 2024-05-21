package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.Set;
import java.util.function.Function;

public class TestCommands {
  public static class TestCommand extends Command {
    final Command command;
    final boolean unitTest;

    public TestCommand(Command command, boolean unitTest) {
      this.command = command;
      this.unitTest = unitTest;
      super.m_requirements = command.getRequirements();
    }

    @Override
    public void initialize() {
      command.initialize();
    }

    @Override
    public void end(boolean interrupted) {
      command.end(interrupted);
    }

    @Override
    public void execute() {
      command.execute();
    }

    @Override
    public boolean isFinished() {
      return command.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
      return command.getRequirements();
    }

    @Override
    public ParallelRaceGroup withTimeout(double seconds) {
      return this.raceWith(unitTest ? new TestWaitCommand(seconds) : new WaitCommand(seconds));
    }

    public static ParallelRaceGroup withTimeout(Command command, boolean unitTest, double seconds) {
      return new TestCommand(command, unitTest).withTimeout(seconds);
    }

    public static Function<Boolean, Command> withTimeout(Command command, double seconds) {
      return u -> new TestCommand(command, u).withTimeout(seconds);
    }
  }

  public static class TestWaitCommand extends Command {
    private double elapsed = 0;
    private final double duration;
    private static final double TICK_RATE = 0.2;

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
  }
}
