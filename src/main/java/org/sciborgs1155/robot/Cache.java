package org.sciborgs1155.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class Cache {
  private static final String cacheFilename = "src/main/deploy/cached_shooter_coeffs.json";
  private static final File cacheFile = new File(Filesystem.getLaunchDirectory(), cacheFilename);

  private static Coeffs coeffs =
      new Coeffs(
          new CoeffsLil(0.001062497903615323, 0.7471887693867607, 5.478684772893152),
          new CoeffsLil(7.752605287137327e-5, -0.007477191156019437, 1.0663329616679225));

  /** desired initial velocity of note, corresponds to pivot angle and flywheel speed */
  public static record NoteTrajectory(
      Rotation2d pivotAngle, double speed, Rotation2d chassisAngle) {
    @Override
    public String toString() {
      return "{pivotAngle: "
          + pivotAngle.getRadians()
          + "; speed: "
          + speed
          + "; chassisAngle: "
          + chassisAngle
          + "}";
    }
  }

  public static void main(String[] args) throws Exception {
    System.exit(runGenCoeffs().exitValue());
  }

  public static Coeffs loadCoeffs() {
    try {
      return loadCoeffsThrows();
    } catch (Exception e) {
      DriverStation.reportError("unable to load coefficients", false);
      return coeffs;
    }
  }

  public static record CoeffsLil(double a, double b, double c) {}

  public static record Coeffs(CoeffsLil launch, CoeffsLil angle) {}

  public static CoeffsLil loadLilCoeffsThrows(JSONObject obj) throws Exception {
    return new CoeffsLil((Double) obj.get("a"), (Double) obj.get("b"), (Double) obj.get("c"));
  }

  public static Coeffs loadCoeffsThrows() throws Exception {
    FileReader reader = new FileReader(cacheFile);
    JSONParser parser = new JSONParser();
    JSONObject obj = (JSONObject) parser.parse(reader);

    return new Coeffs(
        loadLilCoeffsThrows((JSONObject) obj.get("launch")),
        loadLilCoeffsThrows((JSONObject) obj.get("angle")));
  }

  private static Process runGenCoeffs() throws IOException, InterruptedException {
    String pythonPath =
        System.getProperty("os.name").startsWith("Windows") ? "python" : "./venv/bin/python";
    var builder = new ProcessBuilder(pythonPath, "aios" + File.separator + "cache.py");
    builder.redirectOutput(ProcessBuilder.Redirect.INHERIT);
    builder.redirectError(ProcessBuilder.Redirect.INHERIT);

    Process coeffs = builder.start();
    System.out.println("generating coefficients...");
    coeffs.waitFor();
    System.out.println("done");
    return coeffs;
  }

  public static double getVelocity(Translation2d pos) {
    return coeffs.launch.a * pos.getX() + coeffs.launch.b * pos.getY() + coeffs.launch.c;
  }

  public static Rotation2d getPivotAngle(Translation2d pos) {
    return Rotation2d.fromRadians(
        coeffs.angle.a * Math.pow(pos.getX(), 2)
            + coeffs.angle.b * Math.pow(pos.getY(), 2)
            + coeffs.angle.c);
  }

  public static Rotation2d getHeading(Translation2d pos) {
    return Rotation2d.fromRadians(
        pos.getX() == 0 ? Math.PI / 2 : Math.atan(pos.getY() / pos.getX()));
  }

  // TODO have someone make better names
  public static NoteTrajectory getTrajectory(Translation2d pos, Translation2d vel) {
    double v = getVelocity(pos);
    Rotation2d theta = getPivotAngle(pos);
    Rotation2d alpha = getHeading(pos);
    double deltaVx = v * Math.cos(alpha.getRadians()) - vel.getX();
    double deltaVy = v * Math.sin(alpha.getRadians()) - vel.getY();
    Rotation2d returnAlpha =
        Rotation2d.fromRadians(deltaVx == 0 ? Math.PI / 2 : Math.atan(deltaVy / deltaVx));
    Translation2d returnV =
        new Translation2d(deltaVx, deltaVy).times(Math.cos(returnAlpha.getRadians()));
    return new NoteTrajectory(theta, v + returnV.getNorm(), returnAlpha);
  }
}
