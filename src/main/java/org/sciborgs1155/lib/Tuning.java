package org.sciborgs1155.lib;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class Tuning {

  public static DoubleEntry entry(String path, double value) {
    DoubleEntry entry = NetworkTableInstance.getDefault().getDoubleTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }
}
