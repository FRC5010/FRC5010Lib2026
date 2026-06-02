package org.frc5010.common.unit;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import org.frc5010.common.input.DriveVector;
import org.frc5010.common.input.JoystickAxis;
import org.junit.jupiter.api.Test;

/**
 * Layer 1 unit tests for JoystickAxis transforms and DriveVector normalization.
 * No HAL, no WPILib sim — pure Java logic only.
 */
class JoystickAxisTest {

  private static final double EPS = 1e-9;

  // -------------------------------------------------------------------------
  // JoystickAxis — identity
  // -------------------------------------------------------------------------

  @Test
  void identityPassesThrough() {
    assertEquals(0.5, JoystickAxis.of(() -> 0.5).getAsDouble(), EPS);
  }

  @Test
  void identityNegativePassesThrough() {
    assertEquals(-0.75, JoystickAxis.of(() -> -0.75).getAsDouble(), EPS);
  }

  // -------------------------------------------------------------------------
  // JoystickAxis — deadzone
  // -------------------------------------------------------------------------

  @Test
  void deadzoneZerosValueBelowThreshold() {
    assertEquals(0.0, JoystickAxis.of(() -> 0.03).deadzone(0.05).getAsDouble(), EPS);
  }

  @Test
  void deadzoneZerosValueAtThreshold() {
    assertEquals(0.0, JoystickAxis.of(() -> 0.05).deadzone(0.05).getAsDouble(), EPS);
  }

  @Test
  void deadzoneZerosNegativeValueAtThreshold() {
    assertEquals(0.0, JoystickAxis.of(() -> -0.05).deadzone(0.05).getAsDouble(), EPS);
  }

  @Test
  void deadzoneFullDeflectionRemainsOne() {
    assertEquals(1.0, JoystickAxis.of(() -> 1.0).deadzone(0.05).getAsDouble(), EPS);
  }

  @Test
  void deadzoneFullNegativeDeflectionRemainsNegativeOne() {
    assertEquals(-1.0, JoystickAxis.of(() -> -1.0).deadzone(0.05).getAsDouble(), EPS);
  }

  @Test
  void deadzoneRescalesMidpointCorrectly() {
    // input 0.5, threshold 0.1 → rescaled = (0.5 - 0.1) / (1.0 - 0.1) = 0.4/0.9
    double expected = 0.4 / 0.9;
    assertEquals(expected, JoystickAxis.of(() -> 0.5).deadzone(0.1).getAsDouble(), EPS);
  }

  @Test
  void deadzonePreservesSignForNegativeInput() {
    double result = JoystickAxis.of(() -> -0.5).deadzone(0.1).getAsDouble();
    assertTrue(result < 0.0, "Expected negative output for negative input past deadzone");
  }

  // -------------------------------------------------------------------------
  // JoystickAxis — power
  // -------------------------------------------------------------------------

  @Test
  void powerSquaredPositiveInput() {
    assertEquals(0.25, JoystickAxis.of(() -> 0.5).power(2.0).getAsDouble(), EPS);
  }

  @Test
  void powerSquaredPreservesNegativeSign() {
    assertEquals(-0.25, JoystickAxis.of(() -> -0.5).power(2.0).getAsDouble(), EPS);
  }

  @Test
  void powerOneIsIdentity() {
    assertEquals(0.7, JoystickAxis.of(() -> 0.7).power(1.0).getAsDouble(), EPS);
  }

  @Test
  void powerFullDeflectionRemainsOne() {
    assertEquals(1.0, JoystickAxis.of(() -> 1.0).power(3.0).getAsDouble(), EPS);
  }

  // -------------------------------------------------------------------------
  // JoystickAxis — scale
  // -------------------------------------------------------------------------

  @Test
  void scaleMultiplesPositive() {
    assertEquals(2.25, JoystickAxis.of(() -> 0.5).scale(4.5).getAsDouble(), EPS);
  }

  @Test
  void scaleMultiplesNegative() {
    assertEquals(-2.25, JoystickAxis.of(() -> -0.5).scale(4.5).getAsDouble(), EPS);
  }

  @Test
  void scaleByZeroProducesZero() {
    assertEquals(0.0, JoystickAxis.of(() -> 0.9).scale(0.0).getAsDouble(), EPS);
  }

  // -------------------------------------------------------------------------
  // JoystickAxis — limit
  // -------------------------------------------------------------------------

  @Test
  void limitClampsPositiveAboveMax() {
    assertEquals(0.8, JoystickAxis.of(() -> 1.0).limit(0.8).getAsDouble(), EPS);
  }

  @Test
  void limitClampsNegativeBelowNegativeMax() {
    assertEquals(-0.8, JoystickAxis.of(() -> -1.0).limit(0.8).getAsDouble(), EPS);
  }

  @Test
  void limitPassesThroughValueWithinRange() {
    assertEquals(0.5, JoystickAxis.of(() -> 0.5).limit(0.8).getAsDouble(), EPS);
  }

  @Test
  void limitAtExactBoundaryPassesThrough() {
    assertEquals(0.8, JoystickAxis.of(() -> 0.8).limit(0.8).getAsDouble(), EPS);
  }

  // -------------------------------------------------------------------------
  // JoystickAxis — negate
  // -------------------------------------------------------------------------

  @Test
  void negateFlipsPositiveToNegative() {
    assertEquals(-0.7, JoystickAxis.of(() -> 0.7).negate().getAsDouble(), EPS);
  }

  @Test
  void negateFlipsNegativeToPositive() {
    assertEquals(0.3, JoystickAxis.of(() -> -0.3).negate().getAsDouble(), EPS);
  }

  // -------------------------------------------------------------------------
  // JoystickAxis — stacked transforms
  // -------------------------------------------------------------------------

  @Test
  void deadzoneBeforePowerAppliesInOrder() {
    // deadzone(0.1) first, then power(2): 0.5 → rescaled → squared
    double rescaled = (0.5 - 0.1) / (1.0 - 0.1);
    double expected = Math.pow(rescaled, 2.0);
    assertEquals(expected, JoystickAxis.of(() -> 0.5).deadzone(0.1).power(2.0).getAsDouble(), EPS);
  }

  @Test
  void scaleThenLimitCapsOutput() {
    // 0.8 * 2 = 1.6 → clamped to 1.0
    assertEquals(1.0, JoystickAxis.of(() -> 0.8).scale(2.0).limit(1.0).getAsDouble(), EPS);
  }

  @Test
  void negateThenDeadzoneWorksTogether() {
    // negate: -0.03 → deadzone(0.05): within threshold → 0.0
    assertEquals(0.0, JoystickAxis.of(() -> 0.03).negate().deadzone(0.05).getAsDouble(), EPS);
  }

  // -------------------------------------------------------------------------
  // DriveVector — passthrough (no normalization)
  // -------------------------------------------------------------------------

  @Test
  void driveVectorPassesThroughX() {
    assertEquals(0.8, DriveVector.of(() -> 0.8, () -> 0.0).get().getX(), EPS);
  }

  @Test
  void driveVectorPassesThroughY() {
    assertEquals(0.6, DriveVector.of(() -> 0.0, () -> 0.6).get().getY(), EPS);
  }

  @Test
  void driveVectorDiagonalAboveOneNotNormalizedByDefault() {
    Translation2d v = DriveVector.of(() -> 1.0, () -> 1.0).get();
    // Without unitCircle(), magnitude > 1 is preserved
    assertTrue(Math.hypot(v.getX(), v.getY()) > 1.0);
  }

  // -------------------------------------------------------------------------
  // DriveVector — unit-circle normalization
  // -------------------------------------------------------------------------

  @Test
  void unitCircleNormalizesDiagonalToMagnitudeOne() {
    Translation2d v = DriveVector.of(() -> 1.0, () -> 1.0).unitCircle().get();
    assertEquals(1.0, Math.hypot(v.getX(), v.getY()), EPS);
  }

  @Test
  void unitCirclePreservesDirectionOnDiagonal() {
    Translation2d v = DriveVector.of(() -> 1.0, () -> 1.0).unitCircle().get();
    // Normalized diagonal should have equal X and Y components
    assertEquals(v.getX(), v.getY(), EPS);
  }

  @Test
  void unitCircleDoesNotScaleWhenMagnitudeLessThanOne() {
    Translation2d v = DriveVector.of(() -> 0.6, () -> 0.5).unitCircle().get();
    assertEquals(0.6, v.getX(), EPS);
    assertEquals(0.5, v.getY(), EPS);
  }

  @Test
  void unitCircleDoesNotScaleWhenMagnitudeExactlyOne() {
    // Single axis at 1.0, other at 0 → magnitude = 1.0 → unchanged
    Translation2d v = DriveVector.of(() -> 1.0, () -> 0.0).unitCircle().get();
    assertEquals(1.0, v.getX(), EPS);
    assertEquals(0.0, v.getY(), EPS);
  }

  @Test
  void unitCircleHandlesZeroInputSafely() {
    Translation2d v = DriveVector.of(() -> 0.0, () -> 0.0).unitCircle().get();
    assertEquals(0.0, v.getX(), EPS);
    assertEquals(0.0, v.getY(), EPS);
  }

  @Test
  void unitCirclePreservesAxisRatioOnHighMagnitude() {
    // 0.8 and 0.6 → magnitude = 1.0 exactly → unchanged
    Translation2d v = DriveVector.of(() -> 0.8, () -> 0.6).unitCircle().get();
    assertEquals(0.8, v.getX(), EPS);
    assertEquals(0.6, v.getY(), EPS);
  }

  // -------------------------------------------------------------------------
  // DriveVector — deadzone (magnitude-based)
  // -------------------------------------------------------------------------

  @Test
  void driveVectorDeadzoneBelowThresholdReturnsZeroVector() {
    Translation2d v = DriveVector.of(() -> 0.03, () -> 0.03).deadzone(0.1).get();
    assertEquals(0.0, v.getX(), EPS);
    assertEquals(0.0, v.getY(), EPS);
  }

  @Test
  void driveVectorDeadzoneAtThresholdReturnsZeroVector() {
    // exactly on the threshold boundary → snapped to zero
    Translation2d v = DriveVector.of(() -> 0.1, () -> 0.0).deadzone(0.1).get();
    assertEquals(0.0, v.getNorm(), EPS);
  }

  @Test
  void driveVectorDeadzoneFullDeflectionPreservesMagnitudeOne() {
    Translation2d v = DriveVector.of(() -> 1.0, () -> 0.0).deadzone(0.1).get();
    assertEquals(1.0, v.getNorm(), EPS);
  }

  @Test
  void driveVectorDeadzonePreservesDirection() {
    // 45-degree input with magnitude above threshold — direction should remain 45°
    double r = 0.8;
    double comp = r / Math.sqrt(2.0);
    Translation2d v = DriveVector.of(() -> comp, () -> comp).deadzone(0.1).get();
    assertEquals(v.getX(), v.getY(), EPS);
  }

  @Test
  void driveVectorDeadzoneRescalesMagnitudeCorrectly() {
    // magnitude = 0.5, threshold = 0.1 → rescaled = (0.5 - 0.1) / (1.0 - 0.1)
    double input = 0.5;
    double threshold = 0.1;
    double expectedMag = (input - threshold) / (1.0 - threshold);
    Translation2d v = DriveVector.of(() -> input, () -> 0.0).deadzone(threshold).get();
    assertEquals(expectedMag, v.getNorm(), EPS);
  }

  // -------------------------------------------------------------------------
  // DriveVector — power (magnitude-based)
  // -------------------------------------------------------------------------

  @Test
  void driveVectorPowerSquaredReducesMagnitude() {
    // input magnitude = 0.5, power(2) → 0.25
    Translation2d v = DriveVector.of(() -> 0.5, () -> 0.0).power(2.0).get();
    assertEquals(0.25, v.getNorm(), EPS);
  }

  @Test
  void driveVectorPowerPreservesDirection() {
    double comp = 1.0 / Math.sqrt(2.0);
    Translation2d v = DriveVector.of(() -> comp, () -> comp).power(2.0).get();
    assertEquals(v.getX(), v.getY(), EPS);
  }

  @Test
  void driveVectorPowerOneIsIdentity() {
    Translation2d v = DriveVector.of(() -> 0.7, () -> 0.0).power(1.0).get();
    assertEquals(0.7, v.getNorm(), EPS);
  }

  @Test
  void driveVectorPowerFullDeflectionRemainsOne() {
    Translation2d v = DriveVector.of(() -> 1.0, () -> 0.0).power(3.0).get();
    assertEquals(1.0, v.getNorm(), EPS);
  }

  // -------------------------------------------------------------------------
  // DriveVector — scale (magnitude-based)
  // -------------------------------------------------------------------------

  @Test
  void driveVectorScaleDoublesMagnitude() {
    Translation2d v = DriveVector.of(() -> 0.5, () -> 0.0).scale(2.0).get();
    assertEquals(1.0, v.getNorm(), EPS);
  }

  @Test
  void driveVectorScalePreservesDirection() {
    double comp = 0.5 / Math.sqrt(2.0);
    Translation2d v = DriveVector.of(() -> comp, () -> comp).scale(3.0).get();
    assertEquals(v.getX(), v.getY(), EPS);
  }

  @Test
  void driveVectorScaleByZeroReturnsZeroVector() {
    Translation2d v = DriveVector.of(() -> 0.8, () -> 0.6).scale(0.0).get();
    assertEquals(0.0, v.getNorm(), EPS);
  }

  // -------------------------------------------------------------------------
  // DriveVector — limit (magnitude-based)
  // -------------------------------------------------------------------------

  @Test
  void driveVectorLimitClampsMagnitudeAboveMax() {
    // diagonal (1,1) has magnitude √2, limit(1.0) should bring it to 1.0
    Translation2d v = DriveVector.of(() -> 1.0, () -> 1.0).limit(1.0).get();
    assertEquals(1.0, v.getNorm(), EPS);
  }

  @Test
  void driveVectorLimitPassesThroughMagnitudeBelow() {
    Translation2d v = DriveVector.of(() -> 0.5, () -> 0.0).limit(1.0).get();
    assertEquals(0.5, v.getNorm(), EPS);
  }

  @Test
  void driveVectorLimitPreservesDirection() {
    Translation2d v = DriveVector.of(() -> 1.0, () -> 1.0).limit(1.0).get();
    assertEquals(v.getX(), v.getY(), EPS);
  }

  // -------------------------------------------------------------------------
  // DriveVector — negate
  // -------------------------------------------------------------------------

  @Test
  void driveVectorNegateFlipsBothComponents() {
    Translation2d v = DriveVector.of(() -> 0.8, () -> 0.6).negate().get();
    assertEquals(-0.8, v.getX(), EPS);
    assertEquals(-0.6, v.getY(), EPS);
  }

  @Test
  void driveVectorNegatePreservesMagnitude() {
    Translation2d v = DriveVector.of(() -> 0.8, () -> 0.6).negate().get();
    assertEquals(1.0, v.getNorm(), EPS);
  }

  // -------------------------------------------------------------------------
  // DriveVector — chained transforms
  // -------------------------------------------------------------------------

  @Test
  void driveVectorDeadzoneThenPowerAppliesInOrder() {
    // magnitude 0.5, deadzone(0.1): rescaled = 0.4/0.9 ≈ 0.444, then power(2): ≈ 0.198
    double rescaled = (0.5 - 0.1) / (1.0 - 0.1);
    double expected = Math.pow(rescaled, 2.0);
    Translation2d v = DriveVector.of(() -> 0.5, () -> 0.0).deadzone(0.1).power(2.0).get();
    assertEquals(expected, v.getNorm(), EPS);
  }

  @Test
  void driveVectorScaleThenLimitCaps() {
    // magnitude 0.8, scale(2) → 1.6, limit(1.0) → 1.0
    Translation2d v = DriveVector.of(() -> 0.8, () -> 0.0).scale(2.0).limit(1.0).get();
    assertEquals(1.0, v.getNorm(), EPS);
  }

  @Test
  void driveVectorZeroInputThroughTransformChainIsSafe() {
    Translation2d v = DriveVector.of(() -> 0.0, () -> 0.0)
        .deadzone(0.05).power(2.0).scale(4.5).unitCircle().get();
    assertEquals(0.0, v.getX(), EPS);
    assertEquals(0.0, v.getY(), EPS);
  }
}
