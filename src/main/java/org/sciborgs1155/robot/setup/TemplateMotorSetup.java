package org.sciborgs1155.robot.setup;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;

public class TemplateMotorSetup {
  public final Setup createMotor =
      (CANSparkFlex motor, Measure<Current> CURRENT_LIMIT) -> {
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit((int) Math.round((CURRENT_LIMIT.in(Amps))));
      };

  public final Setup createMotorInverted =
      (CANSparkFlex motor, Measure<Current> CURRENT_LIMIT) -> {
        motor.restoreFactoryDefaults();
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit((int) Math.round((CURRENT_LIMIT.in(Amps))));
      };
}
