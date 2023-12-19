package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;

/** Utility class that creates and configures CANSparkMax motor controllers */
public class SparkUtils {

  private static final ArrayList<CANSparkMax> sparks = new ArrayList<>();

  /**
   * Creates a brushless CANSparkMax and restores it to factory defaults.
   *
   * <p>All Spark Max should be created using this method so they can all be stored in a static
   * list.
   *
   * @param id The CAN ID of the Spark Max.
   * @return A CANSparkMax instance.
   * @see #safeBurnFlash()
   */
  public static CANSparkMax create(int id) {
    CANSparkMax spark = new CANSparkMax(id, MotorType.kBrushless);
    spark.restoreFactoryDefaults();
    sparks.add(spark);
    return spark;
  }

  /**
   * Burn all motor configs to flash at the same time, accounting for CAN bus delay. Use once after
   * fully configuring motors.
   */
  public static void safeBurnFlash() {
    Timer.delay(0.2);
    for (CANSparkMax spark : sparks) {
      spark.burnFlash();
      Timer.delay(0.025);
    }
    Timer.delay(0.2);
  }

  /**
   * Disables a list of frames for a specific motor.
   *
   * <p>For a list of specific frames and what they do, read {@href
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces}.
   *
   * <p>Typically, we want to disable as many frames as possible to keep CAN bus usage low.
   * Typically, for all "follower" motor controllers where you do not need to access any data, you
   * should disable frames 1-6.
   *
   * @param spark
   * @param frames
   */
  public static void disableFrames(CANSparkMax spark, int... frames) {
    for (int frame : frames) {
      spark.setPeriodicFramePeriod(PeriodicFrame.fromId(frame), 65535);
    }
  }

  /**
   * Sets the position and velocity conversion factors of a {@link RelativeEncoder} based on the
   * supplied {@link Measure}.
   *
   * <p>The default units of a neo integrated encoder are rotations and minutes. This method
   * automatically converts a supplied ratio into the appropriate units.
   *
   * <pre>
   * encoder = driveMotor.getEncoder();
   * // Configure the encoder to return distance with a gearing of 3/2 and wheel circumference of 2 inches
   * setConversion(encoder, Rotations.of(3).divide(2).times(Inches.of(2)).per(Rotations));
   * </pre>
   *
   * @param encoder The encoder to configure.
   * @param conversion The ratio of rotations to a desired unit as an angle measure.
   */
  public static <U extends Unit<U>> void setConversion(
      RelativeEncoder encoder, Measure<Per<U, Angle>> conversion) {
    var numerator = conversion.unit().numerator();
    encoder.setPositionConversionFactor(conversion.in(numerator.per(Rotations)));
    encoder.setVelocityConversionFactor(
        conversion.per(Seconds.one()).in(numerator.per(Rotations).per(Minute)));
  }

  /**
   * Sets the position and velocity conversion factors of a {@link AbsoluteEncoder} based on the
   * supplied {@link Measure}.
   *
   * <p>The default units of a neo integrated encoder are rotations and minutes. This method
   * automatically converts a supplied ratio into the appropriate units.
   *
   * <pre>
   * encoder = driveMotor.getEncoder();
   * // Configure the encoder to return distance with a gearing of 3/2 and wheel circumference of 2 inches
   * setConversion(encoder, Rotations.of(3).divide(2).times(Inches.of(2)).per(Rotations));
   * </pre>
   *
   * @param encoder The encoder to configure.
   * @param conversion The ratio of rotations to a desired unit as an angle measure.
   */
  public static <U extends Unit<U>> void setConversion(
      AbsoluteEncoder encoder, Measure<Per<U, Angle>> conversion) {
    var numerator = conversion.unit().numerator();
    encoder.setPositionConversionFactor(conversion.in(numerator.per(Rotations)));
    encoder.setVelocityConversionFactor(
        conversion.per(Seconds.one()).in(numerator.per(Rotations).per(Minute)));
  }

  /**
   * Enables and sets the minimum and maximum bounds for input wrapping on an onboard Spark Max PID
   * controller.
   *
   * @param controller The onboard PID controller object.
   * @param min The minimum position input.
   * @param max The maximum position input.
   */
  public static void enableContinuousPIDInput(
      SparkMaxPIDController controller, double min, double max) {
    controller.setPositionPIDWrappingEnabled(true);
    controller.setPositionPIDWrappingMinInput(min);
    controller.setPositionPIDWrappingMaxInput(max);
  }
}
