package org.frc5010.common.drive.swerve.calibration;

/**
 * Immutable result of a motor feedforward calibration fit.
 *
 * <p>Obtained from {@link MotorCalibrationRoutine#fit()} after collecting open-loop
 * voltage-vs-velocity samples. The model is:
 * <pre>
 *   V = kS · sgn(ω) + kV · ω + kA · α
 * </pre>
 * where V is applied voltage (volts), ω is angular velocity (rad/s), and α is angular
 * acceleration (rad/s²). {@code kA} is {@code 0.0} unless identified via a dynamic step test.
 *
 * <p>For torque-current control (TalonFX TorqueCurrentFOC), {@code kS} is expressed in amps
 * and {@code kV}/{@code kA} retain their voltage-mode values — the controller firmware handles
 * the unit conversion internally.
 */
public record CalibrationResult(
    double kS,
    double kV,
    double kA,
    double rSquared) {

  /**
   * Heuristic proportional gain: {@code kP ≈ 0.1 / kV}.
   *
   * <p>This approximates a PID gain that adds roughly 10% of the feedforward voltage per
   * rad/s of velocity error. Use as a starting point; tune on real hardware via
   * {@code TunableGains}.
   */
  public double suggestedKP() {
    return kV > 0 ? 0.1 / kV : 0.0;
  }

  /**
   * Human-readable summary suitable for SmartDashboard, a log message, or console output.
   *
   * <p>Example: {@code kS=0.1234 V, kV=0.0923 V·s/rad, kA=0.0000 V·s²/rad  (R²=0.9987)  → suggested kP=1.0832}
   */
  public String toConfigString() {
    return String.format(
        "kS=%.4f V, kV=%.4f V·s/rad, kA=%.4f V·s²/rad  (R²=%.4f)"
            + "  → suggested kP=%.4f",
        kS, kV, kA, rSquared, suggestedKP());
  }
}
