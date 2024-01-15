package org.sciborgs1155.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged, AutoCloseable {
    private final CANSparkFlex motor;
    
    public Intake() {
        motor = new CANSparkFlex(IntakeConstants.INTAKE_DEVICE_ID, MotorType.kBrushless);
    }
    
    public Command spin(boolean forward) {
        return run(() -> motor.set(forward ? 1 : -1)).finallyDo(motor::disable);   
    }

    public void close() throws Exception {
        motor.close();
    }
}
