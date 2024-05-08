package org.sciborgs1155.lib;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Tuning creates an entry with a specified topic and configurable value in Network Tables.
 *
 * <pre>
 * [Type]Entry entry = Tuning.entry("/[FOLDER_NAME]/[TOPIC_NAME]", [CONFIGURABLE_VALUE]);
 *   // This creates a new configurable datatype value corresponding to the path given and sets the value.
 *
 * Tuning.put([TOPIC], [CONFIGURABLE]);
 *   // This inserts a value under the given topic.
 *
 * ArrayList<[DataType]> arrayList = Tuning.recentChanges([TOPIC]);
 *   // This is returns a list of all of the changes made from a specific topic
 *
 * Tuning.update[DataType]([DataType]Entry);
 *   // This is called periodically in order to check for changes made on Network Tables.
 * </pre>
 */
public final class Tuning {
  /* HashMap of values of each topic path */
  private static HashMap<String, ArrayList<Double>> doubleHash = new HashMap<>();
  private static HashMap<String, ArrayList<Long>> intHash = new HashMap<>();
  private static HashMap<String, ArrayList<String>> stringHash = new HashMap<>();
  private static HashMap<String, ArrayList<Boolean>> booleanHash = new HashMap<>();

  /* Previous values of each topic path */
  private static HashMap<String, Double> prevDouble = new HashMap<>();
  private static HashMap<String, Long> prevInt = new HashMap<>();
  private static HashMap<String, String> prevString = new HashMap<>();
  private static HashMap<String, Boolean> prevBoolean = new HashMap<>();

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

    ArrayList<Double> doubleList = new ArrayList<>();
    doubleList.add(value);

    ArrayList<Double> previousDble = doubleHash.put(path, doubleList);
    if (previousDble == null) {
      previousDble = new ArrayList<>();
      previousDble.add(0.0);
    }
    prevDouble.put(path, previousDble.get(previousDble.size() - 1));

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

    ArrayList<Long> intList = new ArrayList<>();
    intList.add(value);

    ArrayList<Long> previousInt = intHash.put(path, intList);
    if (previousInt == null) {
      previousInt = new ArrayList<>();
      previousInt.add((long) 0);
    }
    prevInt.put(path, previousInt.get(previousInt.size() - 1));

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

    ArrayList<String> strList = new ArrayList<>();

    strList.add(value);

    ArrayList<String> previousStr = stringHash.put(path, strList);
    if (previousStr == null) {
      previousStr = new ArrayList<>();
      previousStr.add("");
    }
    prevString.put(path, previousStr.get(previousStr.size() - 1));

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

    ArrayList<Boolean> boolList = new ArrayList<>();

    boolList.add(value);

    ArrayList<Boolean> previousBool = booleanHash.put(path, boolList);
    if (previousBool == null) {
      previousBool = new ArrayList<>();
      previousBool.add(true);
    }
    prevBoolean.put(path, previousBool.get(previousBool.size() - 1));

    return entry;
  }

  /**
   * Puts a new double value under an already-defined topic on Network Tables
   *
   * @param subtopic The topic under which you want to insert a new value.
   * @param value The value that you want to put under the topic.
   */
  public static void put(DoubleTopic subtopic, double value) {
    DoublePublisher dblePub = subtopic.publish();
    dblePub.accept(value);

    ArrayList<Double> arrayList = new ArrayList<>();
    arrayList.add(value);
    ArrayList<Double> previousDble = doubleHash.put(subtopic.getName(), arrayList);
    if (previousDble == null) {
      previousDble = new ArrayList<>();
      previousDble.add(0.0);
    }
    prevDouble.put(subtopic.getName(), previousDble.get(previousDble.size() - 1));
  }

  /**
   * Puts a int value under an already-defined topic on Network Tables
   *
   * @param subtopic The topic under which you want to insert a new value.
   * @param value The value that you want to put under the topic.
   */
  public static void put(IntegerTopic subtopic, long value) {
    IntegerPublisher intPub = subtopic.publish();
    intPub.accept(value);

    ArrayList<Long> arrayList = new ArrayList<>();
    arrayList.add(value);
    ArrayList<Long> previousInt = intHash.put(subtopic.getName(), arrayList);
    if (previousInt == null) {
      previousInt = new ArrayList<>();
      previousInt.add((long) 0);
    }
    prevInt.put(subtopic.getName(), previousInt.get(previousInt.size() - 1));
  }

  /**
   * Puts a String value under an already-defined topic on Network Tables
   *
   * @param subtopic The topic under which you want to insert a new value.
   * @param value The value that you want to put under the topic.
   */
  public static void put(StringTopic subtopic, String value) {
    StringPublisher strPub = subtopic.publish();
    strPub.accept(value);

    ArrayList<String> arrayList = new ArrayList<>();
    arrayList.add(value);
    ArrayList<String> previousStr = stringHash.put(subtopic.getName(), arrayList);
    if (previousStr == null) {
      previousStr = new ArrayList<>();
      previousStr.add("");
    }
    prevString.put(subtopic.getName(), previousStr.get(previousStr.size() - 1));
  }

  /**
   * Puts a boolean value under an already-defined topic on Network Tables
   *
   * @param subtopic The topic under which you want to insert a new value.
   * @param value The value that you want to put under the topic.
   */
  public static void put(BooleanTopic subtopic, Boolean value) {
    BooleanPublisher boolPub = subtopic.publish();
    boolPub.accept(value);

    ArrayList<Boolean> arrayList = new ArrayList<>();
    arrayList.add(value);
    ArrayList<Boolean> previousBool = booleanHash.put(subtopic.getName(), arrayList);
    if (previousBool == null) {
      previousBool = new ArrayList<>();
      previousBool.add(true);
    }
    prevBoolean.put(subtopic.getName(), previousBool.get(previousBool.size() - 1));
  }

  /**
   * @param topic The topic that you want to get changes from.
   * @return An ArrayList containing all of the changes of the given topic
   */
  public static ArrayList<Double> recentChanges(DoubleTopic topic) {
    if (doubleHash.containsKey(topic.getName())) {
      ArrayList<Double> changes = doubleHash.get(topic.getName());
      return changes;
    }
    return new ArrayList<>();
  }

  /**
   * @param topic The topic that you want to get changes from.
   * @return An ArrayList containing all of the changes of the given topic
   */
  public static ArrayList<Long> recentChanges(IntegerTopic topic) {
    if (intHash.containsKey(topic.getName())) {
      ArrayList<Long> changes = intHash.get(topic.getName());
      return changes;
    }
    return new ArrayList<>();
  }

  /**
   * @param topic The topic that you want to get changes from.
   * @return An ArrayList containing all of the changes of the given topic
   */
  public static ArrayList<String> recentChanges(StringTopic topic) {
    if (stringHash.containsKey(topic.getName())) {
      ArrayList<String> changes = stringHash.get(topic.getName());
      return changes;
    }
    return new ArrayList<>();
  }

  /**
   * @param topic The topic that you want to get changes from.
   * @return An ArrayList containing all of the changes of the given topic
   */
  public static ArrayList<Boolean> recentChanges(BooleanTopic topic) {
    if (booleanHash.containsKey(topic.getName())) {
      ArrayList<Boolean> changes = booleanHash.get(topic.getName());
      return changes;
    }
    return new ArrayList<>();
  }

  /**
   * Updates static records the values of all doubles from a specific entry (use periodically when
   * calling recentChanges()).
   *
   * @param entryList A list of DoubleEntries
   */
  public static void updateDoubles(List<DoubleEntry> entryList) {
    for (int i = 0; i < entryList.size(); i++) {
      String topicName = entryList.get(i).getTopic().getName();

      /* For the circumstance that the given key doesn't exist */
      if (!doubleHash.containsKey(topicName)) {

        ArrayList<Double> arrayList = new ArrayList<>();
        arrayList.add(entryList.get(i).get());

        doubleHash.put(topicName, arrayList);
        prevDouble.put(topicName, arrayList.get(arrayList.size() - 1));
      }

      ArrayList<Double> arrayList = doubleHash.get(topicName);

      if (prevDouble.get(topicName) != entryList.get(i).get()) {

        arrayList.add(entryList.get(i).get());

        /* Updating the previous double value */
        prevDouble.put(topicName, entryList.get(i).get());
      }
    }
  }

  /**
   * Updates static records the values of all ints from a specific entry (use periodically when
   * calling recentChanges()).
   *
   * @param entryList A list of IntegerEntries
   */
  public static void updateInts(List<IntegerEntry> entryList) {
    for (int i = 0; i < entryList.size(); i++) {
      String topicName = entryList.get(i).getTopic().getName();

      /* For the circumstance that the given key doesn't exist */
      if (!intHash.containsKey(topicName)) {

        ArrayList<Long> arrayList = new ArrayList<>();
        arrayList.add(entryList.get(i).get());

        intHash.put(topicName, arrayList);

        prevInt.put(topicName, arrayList.get(arrayList.size() - 1));
      }

      ArrayList<Long> arrayList = intHash.get(topicName);

      if (prevInt.get(topicName) != entryList.get(i).get()) {

        arrayList.add(entryList.get(i).get());

        /* Updating the previous int value */
        prevInt.put(topicName, entryList.get(i).get());
      }
    }
  }

  /**
   * Updates static records the values of all Strings from a specific entry (use periodically when
   * calling recentChanges()).
   *
   * @param entryList A list of StringEntries
   */
  public static void updateStrings(List<StringEntry> entryList) {
    for (int i = 0; i < entryList.size(); i++) {
      String topicName = entryList.get(i).getTopic().getName();

      /* For the circumstance that the given key doesn't exist */
      if (!stringHash.containsKey(topicName)) {

        ArrayList<String> arrayList = new ArrayList<>();
        arrayList.add(entryList.get(i).get());

        stringHash.put(topicName, arrayList);

        prevString.put(topicName, arrayList.get(arrayList.size() - 1));
      }

      ArrayList<String> arrayList = stringHash.get(topicName);

      if (!(prevString.get(topicName).equals(entryList.get(i).get()))) {

        arrayList.add(entryList.get(i).get());

        /* Updating the previous String value */
        prevString.put(topicName, entryList.get(i).get());
      }
    }
  }

  /**
   * Updates static records the values of all booleans from a specific entry (use periodically when
   * calling recentChanges()).
   *
   * @param entryList A list of BooleanEntries
   */
  public static void updateBooleans(List<BooleanEntry> entryList) {
    for (int i = 0; i < entryList.size(); i++) {
      String topicName = entryList.get(i).getTopic().getName();

      /* For the circumstance that the given key doesn't exist */
      if (!booleanHash.containsKey(topicName)) {

        ArrayList<Boolean> arrayList = new ArrayList<>();
        arrayList.add(entryList.get(i).get());

        booleanHash.put(topicName, arrayList);

        prevBoolean.put(topicName, arrayList.get(arrayList.size() - 1));
      }

      ArrayList<Boolean> arrayList = booleanHash.get(topicName);

      if (prevBoolean.get(topicName) != entryList.get(i).get()) {

        arrayList.add(entryList.get(i).get());

        /* Updating the previous boolean value */
        prevBoolean.put(topicName, entryList.get(i).get());
      }
    }
  }
}
