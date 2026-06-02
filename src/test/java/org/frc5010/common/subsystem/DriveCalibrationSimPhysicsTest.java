package org.frc5010.common.subsystem;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.calibration.CalibrationResult;
import org.frc5010.common.drive.swerve.calibration.MotorCalibrationRoutine;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Layer 3 — calibration routine tests backed by a full IronMaple physics world.
 *
 * <p>These tests verify that {@link MotorCalibrationRoutine} produces valid feedforward gains
 * when run against real dyn4j physics, and — critically — that applying those estimated gains as
 * an open-loop feedforward achieves a target velocity within 20% relative error.
 *
 * <p>The per-cycle call order follows the Layer 3 contract:
 * <pre>
 *   drive.runCharacterization(voltage);   // queue voltage to physics controller
 *   drive.simulationPeriodic();           // advance IronMaple (5 × 4 ms sub-ticks)
 *   drive.periodic();                     // read updated physics state
 *   stepOneCycle();                       // advance FPGA clock 20 ms
 * </pre>
 */
class DriveCalibrationSimPhysicsTest extends SimTestBase {

  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  private static final Pose2d SPAWN = new Pose2d(2.0, 2.0, new Rotation2d());

  /** Voltage steps for characterization ramp. */
  private static final double[] VOLTAGE_STEPS = {1.0, 2.0, 3.0, 4.0, 6.0};

  /** Cycles to settle at each voltage step (30 × 20 ms = 600 ms per step). */
  private static final int SETTLE_CYCLES = 30;

  private AkitSwerveDrive drive;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.build(CONSTANTS, SPAWN);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    SimulatedArena.getInstance().shutDown();
    try {
      java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (Exception e) {
      throw new RuntimeException("Failed to reset SimulatedArena singleton", e);
    }
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  /** Convenience: advance one full physics cycle for Layer 3. */
  private void step() {
    drive.simulationPeriodic();
    drive.periodic();
    stepOneCycle();
  }

  // ---------------------------------------------------------------------------
  // Characterization data quality
  // ---------------------------------------------------------------------------

  @Test
  void physicsVoltageRampProducesMonotonicVelocity() {
    enableTeleop();
    double[] velocities = new double[VOLTAGE_STEPS.length];

    for (int i = 0; i < VOLTAGE_STEPS.length; i++) {
      for (int j = 0; j < SETTLE_CYCLES; j++) {
        drive.runCharacterization(VOLTAGE_STEPS[i]);
        step();
      }
      velocities[i] = drive.getDriveFFCharacterizationVelocity();
    }

    for (int i = 1; i < velocities.length; i++) {
      assertTrue(
          velocities[i] > velocities[i - 1],
          "Physics velocity at "
              + VOLTAGE_STEPS[i]
              + " V ("
              + velocities[i]
              + " rad/s) should exceed velocity at "
              + VOLTAGE_STEPS[i - 1]
              + " V ("
              + velocities[i - 1]
              + " rad/s)");
    }
  }

  @Test
  void physicsCalibrationFitHasValidGains() {
    enableTeleop();
    MotorCalibrationRoutine routine =
        MotorCalibrationRoutine.collectDriveRamp(
            drive, VOLTAGE_STEPS, SETTLE_CYCLES, this::step);

    CalibrationResult result = routine.fit();

    assertTrue(result.kV() > 0,
        "kV must be positive; got " + result.kV());
    assertTrue(result.kS() >= 0,
        "kS must be non-negative; got " + result.kS());
    assertTrue(result.rSquared() > 0.85,
        "R² must indicate a good linear fit (> 0.85); got " + result.rSquared());
  }

  // ---------------------------------------------------------------------------
  // End-to-end: calibrated gains achieve target velocity
  // ---------------------------------------------------------------------------

  /**
   * Runs a full calibration cycle:
   * <ol>
   *   <li>Collect open-loop voltage-vs-velocity data from the physics simulation.
   *   <li>Fit kS and kV via least-squares regression.
   *   <li>Compute an open-loop feedforward voltage for a target of 5 rad/s.
   *   <li>Apply that voltage for 50 cycles and measure the steady-state velocity.
   *   <li>Assert the actual velocity is within 20% of the target.
   * </ol>
   *
   * <p>This test demonstrates that the calibration routine produces gains that are
   * directly usable as a feedforward — not merely theoretically valid.
   */
  @Test
  void estimatedGainsAchieveTargetVelocity() {
    enableTeleop();

    MotorCalibrationRoutine routine =
        MotorCalibrationRoutine.collectDriveRamp(
            drive, VOLTAGE_STEPS, SETTLE_CYCLES, this::step);
    CalibrationResult result = routine.fit();

    double targetVelocityRadPerSec = 5.0;
    double ffVoltage = result.kS() + result.kV() * targetVelocityRadPerSec;

    for (int i = 0; i < 50; i++) {
      drive.runCharacterization(ffVoltage);
      step();
    }

    double actualVelocity = drive.getDriveFFCharacterizationVelocity();
    double relativeError = Math.abs(actualVelocity - targetVelocityRadPerSec)
        / targetVelocityRadPerSec;

    assertTrue(
        relativeError < 0.20,
        "Calibrated feedforward should achieve target within 20%; target="
            + targetVelocityRadPerSec
            + " rad/s, actual="
            + actualVelocity
            + " rad/s, error="
            + String.format("%.1f%%", relativeError * 100)
            + "  gains: "
            + result.toConfigString());
  }

  // ---------------------------------------------------------------------------
  // CalibrationResult utility
  // ---------------------------------------------------------------------------

  @Test
  void calibrationResultToStringIsNonEmpty() {
    enableTeleop();
    MotorCalibrationRoutine routine =
        MotorCalibrationRoutine.collectDriveRamp(
            drive, VOLTAGE_STEPS, SETTLE_CYCLES, this::step);

    String summary = routine.fit().toConfigString();

    assertFalse(summary.isBlank(),
        "toConfigString() must return a non-blank string");
    assertTrue(summary.contains("kS"),
        "toConfigString() must include kS label; got: " + summary);
    assertTrue(summary.contains("kV"),
        "toConfigString() must include kV label; got: " + summary);
    assertTrue(summary.contains("R²"),
        "toConfigString() must include R² label; got: " + summary);
  }
}
