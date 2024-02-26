package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.setupHAL;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.FaultLogger.FaultType;

public class FaultLoggerTest {

  @BeforeEach
  public void setup() {
    setupHAL();
    FaultLogger.clear();
  }

  @Test
  void report() {
    NetworkTable base = NetworkTableInstance.getDefault().getTable("Faults");
    var activeInfos =
        base.getSubTable("Active Faults").getStringArrayTopic("infos").subscribe(new String[10]);
    var totalErrors =
        base.getSubTable("Total Faults").getStringArrayTopic("errors").subscribe(new String[10]);
    FaultLogger.update();
    FaultLogger.report("Test", "Example", FaultType.INFO);
    FaultLogger.update();
    assertEquals(FaultLogger.activeFaults().size(), 1);
    assertEquals(FaultLogger.totalFaults().size(), 1);
    assertEquals(activeInfos.get().length, 1);
    assertEquals(totalErrors.get().length, 0);

    // duplicate
    FaultLogger.report("Test", "Example", FaultType.INFO);
    FaultLogger.update();
    assertEquals(FaultLogger.activeFaults().size(), 1);
    assertEquals(FaultLogger.totalFaults().size(), 1);
    assertEquals(activeInfos.get().length, 1);
    assertEquals(totalErrors.get().length, 0);

    FaultLogger.report("Test2", "Example2", FaultType.ERROR);
    FaultLogger.update();
    assertEquals(FaultLogger.activeFaults().size(), 1);
    assertEquals(FaultLogger.totalFaults().size(), 2);
    assertEquals(activeInfos.get().length, 0);
    assertEquals(totalErrors.get().length, 1);
  }

  @Test
  void register() {
    NetworkTable base = NetworkTableInstance.getDefault().getTable("Faults");
    var activeErrors =
        base.getSubTable("Active Faults").getStringArrayTopic("errors").subscribe(new String[10]);
    var totalErrors =
        base.getSubTable("Total Faults").getStringArrayTopic("errors").subscribe(new String[10]);

    FaultLogger.update();
    FaultLogger.register(() -> true, "Recurring Test", "Idk", FaultType.ERROR);
    FaultLogger.update();
    FaultLogger.update();

    // System.out.println(totalErrors.get().toString());
    for (var e : totalErrors.get()) {
      System.out.println(e);
    }

    assertEquals(activeErrors.get().length, 1);
    assertEquals(totalErrors.get().length, 1);
  }
}
