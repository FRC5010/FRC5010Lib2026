package org.frc5010.common.tuning;

/**
 * Groups a set of PID+FF gains as individually tunable NT4 values.
 *
 * <p>All fields are exposed as {@link TunableDouble} instances so they
 * can be edited live from Shuffleboard and logged via AdvantageKit.
 * The {@link #hasChanged()} method returns true if ANY gain changed,
 * making it easy to trigger a single controller update.
 *
 * <p>Usage:
 * <pre>{@code
 * private final TunableGains driveGains =
 *     new TunableGains("Drive", "driveMotor", 0.1, 0.0, 0.0, 0.12);
 *
 * // In periodic():
 * if (driveGains.hasChanged()) {
 *     driveController.setP(driveGains.kP());
 *     driveController.setI(driveGains.kI());
 *     driveController.setD(driveGains.kD());
 *     driveController.setFF(driveGains.kFF());
 * }
 * }</pre>
 */
public class TunableGains {

  private final TunableDouble kP;
  private final TunableDouble kI;
  private final TunableDouble kD;
  private final TunableDouble kFF;

  /**
   * Creates a new TunableGains group.
   *
   * @param table   NT4 table name (e.g. "Drive")
   * @param prefix  prefix for each gain key (e.g. "driveMotor" → "driveMotor_kP")
   * @param kP      initial proportional gain
   * @param kI      initial integral gain
   * @param kD      initial derivative gain
   * @param kFF     initial feedforward gain
   */
  public TunableGains(String table, String prefix,
                      double kP, double kI, double kD, double kFF) {
    this.kP  = new TunableDouble(table, prefix + "_kP",  kP);
    this.kI  = new TunableDouble(table, prefix + "_kI",  kI);
    this.kD  = new TunableDouble(table, prefix + "_kD",  kD);
    this.kFF = new TunableDouble(table, prefix + "_kFF", kFF);
  }

  /** Current proportional gain. */
  public double kP()  { return kP.get(); }

  /** Current integral gain. */
  public double kI()  { return kI.get(); }

  /** Current derivative gain. */
  public double kD()  { return kD.get(); }

  /** Current feedforward gain. */
  public double kFF() { return kFF.get(); }

  /**
   * Returns true if ANY gain changed since the last call.
   * Resets all changed flags as a group — call once per loop.
   */
  public boolean hasChanged() {
    // Evaluate all four — don't short-circuit, so all flags reset.
    boolean p  = kP.hasChanged();
    boolean i  = kI.hasChanged();
    boolean d  = kD.hasChanged();
    boolean ff = kFF.hasChanged();
    return p || i || d || ff;
  }

  /**
   * Directly sets all four gains. Primarily used in tests.
   */
  public void set(double kP, double kI, double kD, double kFF) {
    this.kP.set(kP);
    this.kI.set(kI);
    this.kD.set(kD);
    this.kFF.set(kFF);
  }
}