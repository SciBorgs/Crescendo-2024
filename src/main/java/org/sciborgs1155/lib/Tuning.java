package org.sciborgs1155.lib;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;

/**
 * Tuning creates an entry with a specified topic and configurable value in Network Tables.
 *
 * <pre>
 * Tuning.entry("/[FOLDER_NAME]/[TOPIC_NAME]", [CONFIGURABLE_VALUE]); //this creates a new configurable datatype value corresponding to the path given
 * </pre>
 */
public final class Tuning {

  /**
   * Logs a DoubleEntry on Network Tables.
   *
   * @param path The file path, "/[FOLDER_NAME]/[TOPIC_NAME]", ie: "/Robot/exampleTopicName".
   * @param value The configurable double that you would like to correspond to the topic.
   * @return The DoubleEntry - contains all methods of DoublePublisher, DoubleSubscriber.
   */
  public static DoubleEntry entry(String path, double value) {

    DoubleEntry entry = NetworkTableInstance.getDefault().getDoubleTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }

  /**
   * Logs a IntegerEntry on Network Tables.
   *
   * @param path The file path, "/[FOLDER_NAME]/[TOPIC_NAME]", ie: "/Robot/exampleTopicName".
   * @param value The configurable (int)long that you would like to correspond to the topic.
   * @return The IntegerEntry - contains all methods of IntegerPublisher, IntegerSubscriber.
   */
  public static IntegerEntry entry(String path, long value) {
    IntegerEntry entry = NetworkTableInstance.getDefault().getIntegerTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }

  /**
   * Logs a StringEntry on Network Tables.
   *
   * @param path The file path, "/[FOLDER_NAME]/[TOPIC_NAME]", ie: "/Robot/exampleTopicName".
   * @param value The configurable String that you would like to correspond to the topic.
   * @return The StringEntry - contains all methods of StringPublisher, StringSubscriber.
   */
  public static StringEntry entry(String path, String value) {
    StringEntry entry = NetworkTableInstance.getDefault().getStringTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }

  /**
   * Logs a BooleanEntry on Network Tables.
   *
   * @param path The file path, "/[FOLDER_NAME]/[TOPIC_NAME]", ie: "/Robot/exampleTopicName".
   * @param value The configurable boolean that you would like to correspond to the topic.
   * @return The BooleanEntry - contains all methods of BooleanPublisher, BooleanSubscriber.
   */
  public static BooleanEntry entry(String path, boolean value) {
    BooleanEntry entry = NetworkTableInstance.getDefault().getBooleanTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }
}
