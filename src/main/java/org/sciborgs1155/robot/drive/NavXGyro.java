package org.sciborgs1155.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation3d;
import org.sciborgs1155.lib.FaultLogger;

/** GyroIO implementation for NavX */
public class NavXGyro implements GyroIO {
  private final AHRS ahrs = new AHRS();

  public NavXGyro() {
    FaultLogger.register(ahrs);
  }

  @Override
  public double getRate() {
    return ahrs.getRate();
  }

  @Override
  public Rotation3d getRotation3d() {
    return ahrs.getRotation3d();
  }

  @Override
  public void reset() {
    ahrs.reset();
  }

  @Override
  public void close() throws Exception {}
}
