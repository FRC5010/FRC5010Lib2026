package org.frc5010.common.unit;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frc5010.common.tuning.TunableDouble;
import org.frc5010.common.tuning.TunableGains;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Layer 1 unit tests for TunableDouble and TunableGains.
 *
 * NetworkTables requires HAL initialization, but no simulation
 * hardware is needed — so we init HAL minimally here rather than
 * extending SimTestBase.
 */
class TunableGainsTest {

  @BeforeEach
  void setup() {
    HAL.initialize(500, 0);
    // Use a fresh in-process NT instance for each test to avoid
    // state leaking between tests.
    NetworkTableInstance.getDefault().startLocal();
  }

  @AfterEach
  void teardown() {
    NetworkTableInstance.getDefault().close();
    // Re-create the default instance cleanly for the next test.
    NetworkTableInstance.create();
  }

  // --- TunableDouble ---

  @Test
  void defaultValueReturnedBeforeAnyWrite() {
    TunableDouble t = new TunableDouble("Test", "val", 3.14);
    assertEquals(3.14, t.get(), 1e-9);
  }

  @Test
  void valueUpdatesAfterSet() {
    TunableDouble t = new TunableDouble("Test", "val2", 1.0);
    t.set(2.5);
    assertEquals(2.5, t.get(), 1e-9);
  }

  @Test
  void hasChangedReturnsTrueOnFirstChange() {
    TunableDouble t = new TunableDouble("Test", "val3", 0.0);
    // Consume any initial state
    t.hasChanged();
    t.set(99.0);
    assertTrue(t.hasChanged());
  }

  @Test
  void hasChangedReturnsFalseWhenValueUnchanged() {
    TunableDouble t = new TunableDouble("Test", "val4", 5.0);
    t.hasChanged(); // consume initial
    assertFalse(t.hasChanged());
    assertFalse(t.hasChanged()); // still no change
  }

  @Test
  void hasChangedResetsAfterRead() {
    TunableDouble t = new TunableDouble("Test", "val5", 0.0);
    t.hasChanged();
    t.set(7.0);
    assertTrue(t.hasChanged());
    // Second call — value didn't change again, flag should be false
    assertFalse(t.hasChanged());
  }

  @Test
  void defaultValueAccessibleAfterExternalWrite() {
    TunableDouble t = new TunableDouble("Test", "val6", 42.0);
    assertEquals(42.0, t.getDefault());
    t.set(99.0);
    // Default should be unchanged even after NT write
    assertEquals(42.0, t.getDefault());
  }

  // --- TunableGains ---

  @Test
  void gainsReturnDefaultValuesOnConstruction() {
    TunableGains g = new TunableGains("Drive", "test", 0.1, 0.01, 0.001, 0.12);
    assertEquals(0.1,   g.kP(),  1e-9);
    assertEquals(0.01,  g.kI(),  1e-9);
    assertEquals(0.001, g.kD(),  1e-9);
    assertEquals(0.12,  g.kFF(), 1e-9);
  }

  @Test
  void hasChangedTrueWhenAnyGainChanges() {
    TunableGains g = new TunableGains("Drive", "test2", 0.1, 0.0, 0.0, 0.0);
    g.hasChanged(); // consume initial state
    g.set(0.1, 0.0, 0.0, 0.0); // no change
    assertFalse(g.hasChanged());
    g.set(0.9, 0.0, 0.0, 0.0); // only kP changed
    assertTrue(g.hasChanged());
  }

  @Test
  void hasChangedFalseWhenNoGainChanges() {
    TunableGains g = new TunableGains("Drive", "test3", 1.0, 2.0, 3.0, 4.0);
    g.hasChanged(); // consume
    assertFalse(g.hasChanged());
  }

  @Test
  void allFourGainsUpdateIndependently() {
    TunableGains g = new TunableGains("Drive", "test4", 0.0, 0.0, 0.0, 0.0);
    g.set(1.0, 2.0, 3.0, 4.0);
    assertEquals(1.0, g.kP(),  1e-9);
    assertEquals(2.0, g.kI(),  1e-9);
    assertEquals(3.0, g.kD(),  1e-9);
    assertEquals(4.0, g.kFF(), 1e-9);
  }

  @Test
  void hasChangedDoesNotShortCircuit() {
    // All four flags must be consumed even if the first one is true.
    // This ensures no gain is "stuck changed" after a group read.
    TunableGains g = new TunableGains("Drive", "test5", 0.0, 0.0, 0.0, 0.0);
    g.hasChanged(); // consume initial
    g.set(1.0, 2.0, 3.0, 4.0); // change all four
    assertTrue(g.hasChanged());  // reads and clears all four
    assertFalse(g.hasChanged()); // all clear now
  }
}