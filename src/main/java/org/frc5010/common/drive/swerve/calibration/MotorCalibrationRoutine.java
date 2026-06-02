package org.frc5010.common.drive.swerve.calibration;

import java.util.ArrayList;
import java.util.List;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;

/**
 * Collects open-loop voltage-vs-velocity data from an {@link AkitSwerveDrive} and fits a
 * linear feedforward model via ordinary least-squares regression.
 *
 * <h3>Model</h3>
 * <pre>
 *   V = kS + kV · ω          (for positive ω, kS absorbs static friction)
 * </pre>
 *
 * <h3>Typical usage in a headless test</h3>
 * <pre>
 *   MotorCalibrationRoutine routine = MotorCalibrationRoutine.collectDriveRamp(
 *       drive,
 *       new double[]{1.0, 2.0, 3.0, 4.0, 5.0},  // voltage steps (V)
 *       25,                                         // settle cycles per step
 *       () -> { drive.periodic(); stepOneCycle(); } // Layer-2 cycle
 *   );
 *   CalibrationResult result = routine.fit();
 * </pre>
 *
 * <p>For Layer 3 (IronMaple physics) tests, change the cycle lambda to:
 * <pre>
 *   () -> { drive.simulationPeriodic(); drive.periodic(); stepOneCycle(); }
 * </pre>
 */
public class MotorCalibrationRoutine {

  /**
   * Minimum velocity (rad/s) for a sample to be recorded.
   * Skips dead-band and zero-crossing transients near stall.
   */
  public static final double MIN_VELOCITY_THRESHOLD = 0.1;

  private final List<double[]> samples = new ArrayList<>();

  /**
   * Adds a (voltage, velocity) sample to the collection.
   * Samples where {@code |velocityRadPerSec| < MIN_VELOCITY_THRESHOLD} are silently discarded
   * to avoid polluting the regression with stall-region noise.
   *
   * @param voltageVolts       applied open-loop voltage
   * @param velocityRadPerSec  measured angular velocity (positive)
   */
  public void addSample(double voltageVolts, double velocityRadPerSec) {
    if (Math.abs(velocityRadPerSec) >= MIN_VELOCITY_THRESHOLD) {
      samples.add(new double[]{voltageVolts, velocityRadPerSec});
    }
  }

  /** Returns the number of samples collected so far. */
  public int sampleCount() {
    return samples.size();
  }

  /**
   * Fits the collected samples to {@code V = kS + kV · ω} using ordinary least squares.
   *
   * <p>The regression treats ω (velocity) as the independent variable and V (voltage) as the
   * dependent variable, so the slope is {@code kV} and the intercept is {@code kS}.
   *
   * @return {@link CalibrationResult} with fitted kS, kV, kA=0, and R²; returns a zero result
   *         if fewer than 2 samples were collected
   */
  public CalibrationResult fit() {
    int n = samples.size();
    if (n < 2) {
      return new CalibrationResult(0.0, 0.0, 0.0, 0.0);
    }

    double sumV = 0, sumOmega = 0, sumVOmega = 0, sumOmega2 = 0;
    for (double[] s : samples) {
      double v = s[0];
      double omega = s[1];
      sumV += v;
      sumOmega += omega;
      sumVOmega += v * omega;
      sumOmega2 += omega * omega;
    }

    double denom = n * sumOmega2 - sumOmega * sumOmega;
    if (Math.abs(denom) < 1e-12) {
      return new CalibrationResult(sumV / n, 0.0, 0.0, 0.0);
    }

    double kV = (n * sumVOmega - sumOmega * sumV) / denom;
    double kS = (sumV - kV * sumOmega) / n;

    double rSquared = computeRSquared(kS, kV);

    return new CalibrationResult(Math.max(0.0, kS), kV, 0.0, rSquared);
  }

  private double computeRSquared(double kS, double kV) {
    double meanV = samples.stream().mapToDouble(s -> s[0]).average().orElse(0);
    double ssTot = 0, ssRes = 0;
    for (double[] s : samples) {
      double vActual = s[0];
      double vPredicted = kS + kV * s[1];
      ssTot += (vActual - meanV) * (vActual - meanV);
      ssRes += (vActual - vPredicted) * (vActual - vPredicted);
    }
    return ssTot < 1e-12 ? 1.0 : 1.0 - ssRes / ssTot;
  }

  // ---------------------------------------------------------------------------
  // Static factory helpers for headless tests
  // ---------------------------------------------------------------------------

  /**
   * Runs an open-loop drive voltage ramp and collects steady-state samples.
   *
   * <p>For each entry in {@code voltageSteps}, the method:
   * <ol>
   *   <li>Applies the voltage to all drive motors via {@link AkitSwerveDrive#runCharacterization}
   *       for {@code settleCycles} iterations using {@code cycleStep}.
   *   <li>Records the last measured average drive velocity.
   * </ol>
   *
   * <p>The {@code cycleStep} lambda must advance one 20 ms robot loop. For Layer 2 tests:
   * {@code () -> { drive.periodic(); base.stepOneCycle(); }}.
   * For Layer 3 tests:
   * {@code () -> { drive.simulationPeriodic(); drive.periodic(); base.stepOneCycle(); }}.
   *
   * @param drive        the drive subsystem (must be enabled before calling)
   * @param voltageSteps ascending sequence of open-loop voltages to apply (volts)
   * @param settleCycles number of 20 ms cycles to run at each step before sampling
   * @param cycleStep    Runnable that advances one robot loop (caller-supplied)
   * @return a populated routine ready for {@link #fit()}
   */
  public static MotorCalibrationRoutine collectDriveRamp(
      AkitSwerveDrive drive,
      double[] voltageSteps,
      int settleCycles,
      Runnable cycleStep) {
    MotorCalibrationRoutine routine = new MotorCalibrationRoutine();
    for (double voltage : voltageSteps) {
      for (int i = 0; i < settleCycles; i++) {
        drive.runCharacterization(voltage);
        cycleStep.run();
      }
      double velocity = drive.getDriveFFCharacterizationVelocity();
      routine.addSample(voltage, velocity);
    }
    return routine;
  }

  /**
   * Runs an open-loop steer voltage ramp and collects steady-state samples.
   *
   * <p>Identical to {@link #collectDriveRamp} but uses
   * {@link AkitSwerveDrive#runSteerCharacterization} and
   * {@link AkitSwerveDrive#getSteerFFCharacterizationVelocity()}.
   */
  public static MotorCalibrationRoutine collectSteerRamp(
      AkitSwerveDrive drive,
      double[] voltageSteps,
      int settleCycles,
      Runnable cycleStep) {
    MotorCalibrationRoutine routine = new MotorCalibrationRoutine();
    for (double voltage : voltageSteps) {
      for (int i = 0; i < settleCycles; i++) {
        drive.runSteerCharacterization(voltage);
        cycleStep.run();
      }
      double velocity = drive.getSteerFFCharacterizationVelocity();
      routine.addSample(voltage, velocity);
    }
    return routine;
  }
}
