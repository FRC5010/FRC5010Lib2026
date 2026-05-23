package org.frc5010.common.tuning;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import org.littletonrobotics.junction.Logger;

/**
 * A double value that can be tuned live via NetworkTables (NT4) and is
 * automatically logged via AdvantageKit on every change.
 *
 * <p>In REAL and SIM modes, the value is published to NT4 under
 * "/Tuning/{table}/{key}" and can be edited from Shuffleboard or
 * AdvantageScope. In REPLAY mode the value is read from the log and
 * NT4 writes are ignored, preserving deterministic replay.
 *
 * <p>Usage:
 * <pre>{@code
 * private final TunableDouble driveKp =
 *     new TunableDouble("Drive", "driveKp", 0.1);
 *
 * // In periodic():
 * if (driveKp.hasChanged()) {
 *     driveController.setP(driveKp.get());
 * }
 * }</pre>
 */
public class TunableDouble {

  private static final String TABLE_PREFIX = "/Tuning/";

  private final String logKey;
  private final NetworkTableEntry entry;
  private final double defaultValue;
  private double lastValue;
  private boolean hasChanged = false;

  /**
   * Creates a new TunableDouble.
   *
   * @param table  logical grouping shown in NT4/Shuffleboard (e.g. "Drive")
   * @param key    name of this value within the table (e.g. "driveKp")
   * @param defaultValue  value used before any NT4 write occurs
   */
  public TunableDouble(String table, String key, double defaultValue) {
    this.defaultValue = defaultValue;
    this.lastValue = defaultValue;
    this.logKey = TABLE_PREFIX + table + "/" + key;

    entry = NetworkTableInstance.getDefault()
        .getTable(TABLE_PREFIX + table)
        .getEntry(key);

    // Publish the default so it shows up in Shuffleboard immediately.
    // If a value already exists (e.g. from a previous run), this is a no-op.
    entry.setDefaultDouble(defaultValue);
  }

  /**
   * Returns the current value. If NT4 has not been written to, returns
   * the default value provided at construction.
   */
  public double get() {
    return entry.getDouble(defaultValue);
  }

  /**
   * Returns true if the value has changed since the last time this method
   * was called. Resets the changed flag on each call.
   *
   * <p>Typical usage: call once per periodic loop, update controllers only
   * when this returns true.
   */
  public boolean hasChanged() {
    double current = get();
    if (current != lastValue) {
      lastValue = current;
      Logger.recordOutput(logKey, current);
      hasChanged = true;
    } else {
      hasChanged = false;
    }
    return hasChanged;
  }

  /**
   * Returns the default value this TunableDouble was constructed with.
   * Useful for resetting to known-good values in tests.
   */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Programmatically sets the NT4 value. Primarily used in tests to
   * simulate a user editing the value from Shuffleboard.
   */
  public void set(double value) {
    entry.setDouble(value);
  }
}