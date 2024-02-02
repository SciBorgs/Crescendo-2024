package org.sciborgs1155.lib;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;

public class TalonUtils {
  private static final Orchestra orchestra = new Orchestra();
  private static final ArrayList<TalonFX> talons = new ArrayList<>(4);
  private static boolean fileLoaded = false;

  /**
   * Adds motor to the orchestra.
   *
   * @param talon The motor to add.
   */
  public static void addMotor(TalonFX talon) {
    talons.add(talon);
  }

  /**
   * Configure all motors to play a selected Chirp (CHRP) file in the deploy directory. Should be
   * called once after addition of all Talons to TalonUtils.
   *
   * <p>Use {@code loadOrchestraFile()} after configuration to change the played file.
   *
   * @param fileName The path of the file to play.
   * @return Whether loading the file was successful.
   */
  public static boolean configureOrchestra(String fileName) {
    AudioConfigs audioCfg = new AudioConfigs().withAllowMusicDurDisable(true);
    for (TalonFX talon : talons) {
      talon.getConfigurator().apply(audioCfg);
      orchestra.addInstrument(talon);
    }
    return loadOrchestraFile(fileName);
  }

  /**
   * Load the selected CHRP file located in the deploy directory.
   *
   * @param fileName The name of the file to play.
   * @return Whether loading the file was successful.
   */
  public static boolean loadOrchestraFile(String fileName) {
    var success = orchestra.loadMusic(fileName);
    fileLoaded = success.isOK();
    return fileLoaded;
  }

  /**
   * Begin playback of the loaded file.
   *
   * @return Whether the operation was successful.
   */
  public static boolean play() {
    var success = orchestra.play();
    return success.isOK();
  }

  /**
   * Stop and restart playback of the loaded file.
   *
   * @return Whether the operation was successful.
   */
  public static boolean stop() {
    var success = orchestra.stop();
    return success.isOK();
  }

  /**
   * Pause playback of the loaded file.
   *
   * @return Whether the operation was successful.
   */
  public static boolean pause() {
    var success = orchestra.pause();
    return success.isOK();
  }
}
