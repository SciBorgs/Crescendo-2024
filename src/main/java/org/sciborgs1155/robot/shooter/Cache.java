package org.sciborgs1155.robot.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Hashtable;
import java.util.Map;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class Cache {
  private static final String cacheFilename = "shooter_states_cache.json";
  private static Hashtable<Translation2d, ShooterState> data = new Hashtable<>();

  /** desired initial velocity of note, corresponds to pivot angle and flywheel speed */
  public static record ShooterState(Rotation2d angle, double speed) {
    @Override
    public String toString() {
      return "{angle: " + angle.getRadians() + "; speed: " + speed + "}";
    }
  }

  private static Translation2d strToPoint(String str) {
    String[] asStrs = str.split(",");
    return new Translation2d(Double.parseDouble(asStrs[0]), Double.parseDouble(asStrs[1]));
  }

  private static ShooterState dictToState(JSONObject dict) {
    return new ShooterState(
        Rotation2d.fromRadians((double) dict.get("angle")), (double) dict.get("speed"));
  }

  public static Hashtable<Translation2d, ShooterState> loadStates() {
    try {
      return loadStatesThrows();
    } catch (Exception e) {
      return new Hashtable<>();
    }
  }

  @SuppressWarnings("unchecked")
  private static Hashtable<Translation2d, ShooterState> loadStatesThrows()
      throws IOException, InterruptedException, org.json.simple.parser.ParseException {
    FileReader reader = new FileReader(new File(Filesystem.getDeployDirectory(), cacheFilename));
    JSONParser parser = new JSONParser();
    JSONObject obj = (JSONObject) parser.parse(reader);

    obj.forEach(
        (k, v) -> {
          if (k instanceof String && v instanceof JSONObject) {
            data.put(strToPoint((String) k), dictToState((JSONObject) v));
          } else {
            throw new RuntimeException("something has gone wroooonnnnggg!!");
          }
        });

    return data;
  }

  public static String dataToString(Hashtable<Translation2d, ShooterState> d) {
    String str = "";
    for (Map.Entry<Translation2d, ShooterState> entry : d.entrySet()) {
      var p = entry.getKey();
      var s = entry.getValue();
      str += "(" + p.getX() + ", " + p.getY() + ") : " + s + "\n";
    }
    return str;
  }

  private static Process runCasadi() throws IOException, InterruptedException {
    String pythonPath =
        System.getProperty("os.name").startsWith("Windows") ? "python" : "./venv/bin/python";
    var builder = new ProcessBuilder(pythonPath, "aios" + File.separator + "cache.py");
    builder.redirectOutput(ProcessBuilder.Redirect.INHERIT);
    builder.redirectError(ProcessBuilder.Redirect.INHERIT);

    Process casadi = builder.start();
    System.out.println("generating states...");
    casadi.waitFor();
    return casadi;
  }

  public static void main(String[] args) throws Exception {
    System.exit(runCasadi().exitValue());
  }
}
