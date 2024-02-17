package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Ports.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

import java.util.List;
import java.util.Set;
import org.ejml.simple.SimpleMatrix;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.Constants;

public class RealPivot implements PivotIO {
  private final CANSparkMax lead;
  private final CANSparkMax leftBottom;
  private final CANSparkMax rightTop;
  private final CANSparkMax rightBottom;
  private final RelativeEncoder encoderLT;
  private final RelativeEncoder encoderLB;
  private final RelativeEncoder encoderRT;
  private final RelativeEncoder encoderRB;
  // idk where the 5th encoder is
  private final LinearSystem<N2, N1, N1> pivotModelPosition =
      LinearSystemId.identifyPositionSystem(kV, 0);
  private final KalmanFilter<N2, N1, N1> kalmanPivotPosition =
      new KalmanFilter<N2, N1, N1>(
          Nat.N2(),
          Nat.N1(),
          pivotModelPosition,
          VecBuilder.fill(3.0, 3.0), // starting estimate? (position)
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

    encoderLT = lead.getAlternateEncoder(SparkUtils.THROUGHBORE_CPR);
    encoderLB = leftBottom.getAlternateEncoder(SparkUtils.THROUGHBORE_CPR);
    encoderRT = rightBottom.getAlternateEncoder(SparkUtils.THROUGHBORE_CPR);
    encoderRB = rightTop.getAlternateEncoder(SparkUtils.THROUGHBORE_CPR);

    for (RelativeEncoder encoder : List.of(encoderLT, encoderLB, encoderRT, encoderRB)) {
      encoder.setPositionConversionFactor(POSITION_FACTOR.in(Radians));
      encoder.setVelocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond));
    }

    SparkUtils.configureFrameStrategy(
        lead, Set.of(Data.POSITION, Data.VELOCITY, Data.OUTPUT), Set.of(Sensor.QUADRATURE), true);
    SparkUtils.configureNothingFrameStrategy(leftBottom);
    SparkUtils.configureNothingFrameStrategy(rightTop);
    SparkUtils.configureNothingFrameStrategy(rightBottom);

    lead.burnFlash();
    leftBottom.burnFlash();
    rightTop.burnFlash();
    rightBottom.burnFlash();

    kalmanPivotPosition.reset();
  }

  @Override
  public void setVoltage(double voltage) {
    double[][] measurement =
      {
        {encoderLT.getPosition(), encoderLB.getPosition()},
        {encoderRT.getPosition(), encoderRB.getPosition()}
      };

    double[][] controllerVoltage = {{voltage}};
    SimpleMatrix inputMatrix = new SimpleMatrix(measurement);
    SimpleMatrix inputVoltage = new SimpleMatrix(controllerVoltage);

    kalmanPivotPosition.predict(VecBuilder.fill(voltage), Constants.PERIOD.in(Seconds));
    kalmanPivotPosition.correct(
      VecBuilder.fill(voltage), 
      // inputMatrix.getMatrix()
      VecBuilder.fill(
        
          encoderLT.getPosition(), 
          encoderLB.getPosition(), 
          encoderRT.getPosition(), 
          encoderRB.getPosition()
        
        )
    );

    lead.setVoltage(voltage);
  }

  @Override
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(encoderLT.getPosition());
  }

  @Override
  public double getVelocity() {
    return encoderLT.getVelocity();
  }

  @Override
  public void close() throws Exception {
    lead.close();
    leftBottom.close();
    rightTop.close();
    rightBottom.close();
  }
}
