package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.setupTests;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import java.util.Set;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class SparkUtilsTest {

  @BeforeEach
  public void setup() {
    setupTests();
  }

  @Test
  void configure() {
    CANSparkFlex motor = new CANSparkFlex(1, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();

    SparkUtils.configure(
        motor,
        () ->
            SparkUtils.configureFrameStrategy(
                motor,
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.INTEGRATED),
                false),
        () -> motor.setIdleMode(IdleMode.kBrake),
        () -> motor.setSmartCurrentLimit(30),
        () -> encoder.setPositionConversionFactor(0.5),
        () -> encoder.setVelocityConversionFactor(0.25),
        () -> encoder.setMeasurementPeriod(8),
        () -> encoder.setAverageDepth(2));

    assertEquals(IdleMode.kBrake, motor.getIdleMode());
    assertEquals(0.5, encoder.getPositionConversionFactor());
    assertEquals(0.25, encoder.getVelocityConversionFactor());
    assertEquals(8, encoder.getMeasurementPeriod());
    assertEquals(2, encoder.getAverageDepth());

    motor.close();
  }
}
