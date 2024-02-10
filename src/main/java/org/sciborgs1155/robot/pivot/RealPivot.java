package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;

import java.util.List;
import java.util.Set;

import org.ejml.simple.SimpleMatrix;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealPivot implements PivotIO {
  private final CANSparkMax lead;
  private final CANSparkMax leftBottom;
  private final CANSparkMax rightTop;
  private final CANSparkMax rightBottom;
  private final SparkAbsoluteEncoder encoderLT;
  private final SparkAbsoluteEncoder encoderLB;
  private final SparkAbsoluteEncoder encoderRT;
  private final SparkAbsoluteEncoder encoderRB;
  // idk where the 5th encoder is
  private final LinearSystem<N1, N1, N1> pivotModelPosition = new LinearSystem<>(null, null, null, null);
  private final KalmanFilter<N1, N1, N1> sharedMotorsKalmanPosition =
    new KalmanFilter<>(
        Nat.N1(),
        Nat.N1(),
        pivotModelPosition,
        VecBuilder.fill(3.0), // starting estimate? (position)
        VecBuilder.fill(0.01), // noise estimate (poition)
        0.020);

  private final LinearSystem<N1, N1, N1> pivotModelVelocity = new LinearSystem<>(null, null, null, null);
  private final KalmanFilter<N1, N1, N1> sharedMotorsKalmanVelocity =
    new KalmanFilter<>(
        Nat.N1(),
        Nat.N1(),
        pivotModelVelocity,
        VecBuilder.fill(3.0), // starting estimate? (position)
        VecBuilder.fill(0.01), // noise estimate (poition)
        0.020);

  public RealPivot() {
    lead = new CANSparkMax(SPARK_LEFT_TOP, MotorType.kBrushless);
    leftBottom = new CANSparkMax(SPARK_LEFT_BOTTOM, MotorType.kBrushless);
    rightTop = new CANSparkMax(SPARK_RIGHT_TOP, MotorType.kBrushless);
    rightBottom = new CANSparkMax(SPARK_RIGHT_BOTTOM, MotorType.kBrushless);

    for (CANSparkMax spark : List.of(lead, leftBottom, rightTop, rightBottom)) {
      spark.restoreFactoryDefaults();
      spark.setIdleMode(IdleMode.kBrake);
      spark.setSmartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    }

    lead.setInverted(false);
    leftBottom.follow(lead, false);
    rightTop.follow(lead, true);
    rightBottom.follow(lead, true);

    encoderLT = lead.getAbsoluteEncoder(Type.kDutyCycle);
    encoderLB = leftBottom.getAbsoluteEncoder(Type.kDutyCycle);
    encoderRT = rightTop.getAbsoluteEncoder(Type.kDutyCycle);
    encoderRB = rightBottom.getAbsoluteEncoder(Type.kDutyCycle);

    for (SparkAbsoluteEncoder encoder : List.of(encoderLT, encoderLB, encoderRT, encoderRB)) {
      encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians));
      encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond));
    }

    SparkUtils.configureFrameStrategy(
        lead, Set.of(Data.POSITION, Data.VELOCITY, Data.OUTPUT), Set.of(Sensor.DUTY_CYCLE), true);
    SparkUtils.configureFollowerFrameStrategy(leftBottom);
    SparkUtils.configureFollowerFrameStrategy(rightTop);
    SparkUtils.configureFollowerFrameStrategy(rightBottom);

    lead.burnFlash();
    leftBottom.burnFlash();
    rightTop.burnFlash();
    rightBottom.burnFlash();
  }

  @Override
  public void setVoltage(double voltage) {
    lead.setVoltage(voltage);
  }

  @Override
  public Rotation2d getPosition() {
    double[][] array = {{encoderLT.getPosition() + encoderLB.getPosition() + encoderRT.getPosition() + encoderRB.getPosition()}};
    SimpleMatrix matrix = new SimpleMatrix(array);
        
    sharedMotorsKalmanPosition.predict(matrix.getMatrix(), 0.02);
    double[][] measurement = {{1}};
    sharedMotorsKalmanPosition.correct(matrix.getMatrix(), (new SimpleMatrix(measurement)).getMatrix());

    return Rotation2d.fromRadians(sharedMotorsKalmanPosition.getP(0, 0)/4);
  }

  @Override
  public double getVelocity() {
    double[][] array = {{encoderLT.getVelocity() + encoderLB.getVelocity() + encoderRT.getVelocity() + encoderRB.getVelocity()}};
    SimpleMatrix matrix = new SimpleMatrix(array);

    sharedMotorsKalmanVelocity.predict(matrix.getMatrix(), 0.02);
    double[][] measurement = {{1}};
    sharedMotorsKalmanVelocity.correct(matrix.getMatrix(), (new SimpleMatrix(measurement)).getMatrix());

    return sharedMotorsKalmanVelocity.getP(0, 0)/4;
  }

  @Override
  public void close() throws Exception {
    lead.close();
    leftBottom.close();
    rightTop.close();
    rightBottom.close();
  }
}
