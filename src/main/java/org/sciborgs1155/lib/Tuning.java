package org.sciborgs1155.lib;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;

public final class Tuning {

  public static DoubleEntry entry(String path, double value) {
    DoubleEntry entry = NetworkTableInstance.getDefault().getDoubleTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }

  public static IntegerEntry entry(String path, long value) {
    IntegerEntry entry = NetworkTableInstance.getDefault().getIntegerTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }

  public static StringEntry entry(String path, String value) {
    StringEntry entry = NetworkTableInstance.getDefault().getStringTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }

  public static BooleanEntry entry(String path, boolean value) {
    BooleanEntry entry = NetworkTableInstance.getDefault().getBooleanTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }
}
