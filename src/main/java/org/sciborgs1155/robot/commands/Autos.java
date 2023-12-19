package org.sciborgs1155.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public final class Autos implements Sendable {

  private final SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();

  public Command get() {
    return chooser.getSelected().get();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    chooser.initSendable(builder);
  }
}
