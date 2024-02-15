package org.sciborgs1155.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.BufferedReader;
import java.io.File;
import java.io.InputStreamReader;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class Cache {
  // launch coefficients
  private static double av = 0.001062497903615323;
  private static double bv = 0.7471887693867607;
  private static double cv = 5.478684772893152;
  // angle coefficients
  private static double at = 7.752605287137327e-5;
  private static double bt = -0.007477191156019437;
  private static double ct = 1.0663329616679225;

  public static void main(String[] args) throws Exception {
    System.exit(reloadCoeffs());
  }

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

  private static JSONObject parseString(String s) throws ParseException {
    String jsonValue = "";
    for (int i = 0; i < s.length(); i++) {
      var c = s.charAt(i);
      jsonValue += (c == '\'') ? "\"" : c;
    }
    return (JSONObject) (new JSONParser().parse(jsonValue));
  }

  /**
   * @return exit status
   */
  private static int reloadCoeffs() {
    try {
      int n = reloadCoeffsThrows();
      System.out.println("reloaded coeffs");
      return n;
    } catch (Exception e) {
      System.out.println(
          "couldn't load coeffs from script!! using defaults. error: " + e.getMessage());
      return 0;
    }
  }

  /**
   * @return python process exit status
   */
  private static int reloadCoeffsThrows() throws Exception {
    String pythonPath =
        System.getProperty("os.name").startsWith("Windows") ? "python" : "./venv/bin/python";
    var builder = new ProcessBuilder(pythonPath, "aios" + File.separator + "cache.py");
    builder.redirectError(ProcessBuilder.Redirect.INHERIT);
    Process coeffs = builder.start();
    var in = new BufferedReader(new InputStreamReader(coeffs.getInputStream()));

    String output;
    while ((output = in.readLine()) == null) {}

    var obj = parseString(output);
    av = (Double) obj.get("av");
    bv = (Double) obj.get("bv");
    cv = (Double) obj.get("cv");
    at = (Double) obj.get("at");
    bt = (Double) obj.get("bt");
    ct = (Double) obj.get("ct");

    return coeffs.waitFor();
  }

  public static double getVelocity(Translation2d pos) {
    return av * pos.getX() + bv * pos.getY() + cv;
  }

  public static Rotation2d getPivotAngle(Translation2d pos) {
    return Rotation2d.fromRadians(at * Math.pow(pos.getX(), 2) + bt * Math.pow(pos.getY(), 2) + ct);
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
