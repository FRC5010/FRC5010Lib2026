package org.frc5010.common.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Provides the current robot operating mode as a single, consistent value
 * across the entire framework.
 *
 * <p>The mode is determined once at startup:
 * <ul>
 *   <li>On a real roboRIO: REAL</li>
 *   <li>In WPILib simulation: SIM (unless overridden to REPLAY)</li>
 *   <li>When replaying an AKit log: REPLAY (set manually before robot init)</li>
 * </ul>
 *
 * <p>Students set the mode in their Robot.java constructor before Logger.start():
 * <pre>{@code
 * // In Robot.java constructor, before Logger.start():
 * if (RobotBase.isReal()) {
 *     RobotMode.set(Mode.REAL);
 * } else if (isReplay) {
 *     RobotMode.set(Mode.REPLAY);
 * } else {
 *     RobotMode.set(Mode.SIM);
 * }
 * }</pre>
 */
public class RobotMode {

  private static Mode current = null;

  /** Sets the active mode. Must be called before any subsystem is constructed. */
  public static void set(Mode mode) {
    if (mode == null) throw new IllegalArgumentException("Mode must not be null");
    current = mode;
  }

  /**
   * Returns the current mode. Throws if called before {@link #set(Mode)},
   * which catches initialization order bugs at startup rather than silently
   * using the wrong IO implementation.
   */
  public static Mode get() {
    if (current == null) {
      throw new IllegalStateException(
          "RobotMode has not been set. Call RobotMode.set() in Robot.java " +
          "before constructing any subsystems.");
    }
    return current;
  }

  /** Convenience: true when running on real hardware. */
  public static boolean isReal()   { return get() == Mode.REAL; }

  /** Convenience: true when running in simulation. */
  public static boolean isSim()    { return get() == Mode.SIM; }

  /** Convenience: true when replaying a log file. */
  public static boolean isReplay() { return get() == Mode.REPLAY; }

  /**
   * Resets the mode to unset. Only for use in unit tests — never call
   * this in production code.
   */
  public static void resetForTesting() {
    current = null;
  }
}