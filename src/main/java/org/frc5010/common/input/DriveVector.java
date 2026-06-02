package org.frc5010.common.input;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.UnaryOperator;

/**
 * Combines two joystick axes into a 2-D drive vector with a chainable transform pipeline.
 *
 * <p>Transforms operate on the vector's <em>magnitude</em> while preserving its direction,
 * mirroring the transform API of {@link JoystickAxis}. Use {@link #unitCircle()} (an alias
 * for {@link #limit(double) limit(1.0)}) to prevent diagonal inputs from exceeding 1.
 *
 * <p>Example — deadzone and response curve applied to the combined magnitude, then clamped:
 * <pre>{@code
 * DriveVector translate = DriveVector.of(
 *     controller.axis(1).negate(),
 *     controller.axis(0).negate()
 * ).deadzone(0.05).power(2.0).unitCircle();
 *
 * // Inside the drive command lambda:
 * Translation2d xy = translate.get();
 * double vx = xy.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond);
 * double vy = xy.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond);
 * }</pre>
 */
public class DriveVector {

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private UnaryOperator<Translation2d> chain;

  private DriveVector(DoubleSupplier x, DoubleSupplier y) {
    this.xSupplier = x;
    this.ySupplier = y;
    this.chain = v -> v;
  }

  /**
   * Creates a drive vector from two axis suppliers.
   *
   * @param x the first axis (maps to {@link Translation2d#getX()})
   * @param y the second axis (maps to {@link Translation2d#getY()})
   */
  public static DriveVector of(DoubleSupplier x, DoubleSupplier y) {
    return new DriveVector(x, y);
  }

  /**
   * Apply a deadzone to the vector's magnitude: if magnitude is {@code ≤ threshold}
   * the vector becomes zero. The remaining range is rescaled so full deflection still
   * maps to magnitude 1.
   */
  public DriveVector deadzone(double threshold) {
    return appendMagnitude(mag -> {
      if (mag <= threshold) return 0.0;
      return (mag - threshold) / (1.0 - threshold);
    });
  }

  /**
   * Raise the vector's magnitude to {@code exponent}.
   * e.g. {@code power(2)} gives a squared (gentle-start) response curve.
   */
  public DriveVector power(double exponent) {
    return appendMagnitude(mag -> Math.pow(mag, exponent));
  }

  /** Multiply the vector's magnitude by {@code factor}. */
  public DriveVector scale(double factor) {
    return appendMagnitude(mag -> mag * factor);
  }

  /** Clamp the vector's magnitude to {@code [0, max]}. */
  public DriveVector limit(double max) {
    return appendMagnitude(mag -> Math.min(max, mag));
  }

  /**
   * Clamp the vector's magnitude to 1.0, preventing diagonal inputs from exceeding
   * the robot's maximum speed. Equivalent to {@code limit(1.0)}.
   */
  public DriveVector unitCircle() {
    return limit(1.0);
  }

  /** Reverse the direction of the vector (negates both X and Y components). */
  public DriveVector negate() {
    return append(v -> new Translation2d(-v.getX(), -v.getY()));
  }

  /**
   * Evaluate both axes and return the transformed vector.
   * The returned {@link Translation2d} has X = first axis, Y = second axis.
   */
  public Translation2d get() {
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();
    return chain.apply(new Translation2d(x, y));
  }

  private DriveVector appendMagnitude(DoubleUnaryOperator op) {
    return append(v -> {
      double mag = v.getNorm();
      if (mag == 0.0) return new Translation2d();
      double newMag = op.applyAsDouble(mag);
      return new Translation2d(v.getX() / mag * newMag, v.getY() / mag * newMag);
    });
  }

  private DriveVector append(UnaryOperator<Translation2d> next) {
    UnaryOperator<Translation2d> prev = chain;
    chain = v -> next.apply(prev.apply(v));
    return this;
  }
}
