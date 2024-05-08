package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.setupTests;

import java.util.ArrayList;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.StringEntry;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.Tuning.*;
import org.sciborgs1155.robot.drive.Drive;

public class TuningTest {

  @BeforeEach
  public void setup() {
    setupTests();
  }

  @Test
  void fullEntryTest() {
    DoubleEntry dbleEnt;
    IntegerEntry intEnt;
    StringEntry strEnt;
    BooleanEntry boolEnt;

    double dbleVal = 2.0;
    long intVal = 7823; // IntgerTopic.getEntry() accepts longs for default values
    String strVal = "Hello, World! <3";
    boolean boolVal = true;

    dbleEnt = Tuning.entry("/Robot/a", dbleVal);
    intEnt = Tuning.entry("/Robot/b", intVal);
    strEnt = Tuning.entry("/Robot/c", strVal);
    boolEnt = Tuning.entry("/Robot/d", boolVal);

    assertEquals(dbleVal, dbleEnt.get());
    assertEquals(intVal, intEnt.get());
    assertEquals(strVal, strEnt.get());
    assertEquals(boolVal, boolEnt.get());

    Tuning.put(dbleEnt.getTopic(), 22930209.13);
    Tuning.put(intEnt.getTopic(), 123456789);
    Tuning.put(strEnt.getTopic(), "como estas");
    Tuning.put(boolEnt.getTopic(), false);

    assertEquals(22930209.13, dbleEnt.get());
    assertEquals(123456789, intEnt.get());
    assertEquals("como estas", strEnt.get());
    assertEquals(false, boolEnt.get());

    ArrayList<Double> doubleList = new ArrayList();
    doubleList.add(dbleVal);
    doubleList.add(22930209.13);

    ArrayList<Long> intList = new ArrayList();
    intList.add(intVal);
    intList.add((long)123456789);

    ArrayList<String> strList = new ArrayList();
    strList.add(strVal);
    strList.add("como estas");

    ArrayList<Boolean> boolList = new ArrayList<>();
    boolList.add(boolVal);
    boolList.add(false);

    assertEquals(doubleList, Tuning.recentChanges(dbleEnt.getTopic()));
    assertEquals(intList, Tuning.recentChanges(intEnt.getTopic()));
    assertEquals(strList, Tuning.recentChanges(strEnt.getTopic()));
    assertEquals(boolList, Tuning.recentChanges(boolEnt.getTopic()));

    assertEquals(doubleList.get(1), Tuning.recentChanges(dbleEnt.getTopic(), 1).get(0));
    assertEquals(intList.get(1), Tuning.recentChanges(intEnt.getTopic(), 1).get(0));
    assertEquals(strList.get(1), Tuning.recentChanges(strEnt.getTopic(), 1).get(0));
    assertEquals(boolList.get(1), Tuning.recentChanges(boolEnt.getTopic(), 1).get(0));
  }
}
