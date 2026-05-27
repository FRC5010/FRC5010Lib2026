package org.frc5010.common.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Immutable per-camera configuration. Use the {@link Builder} to construct instances.
 *
 * <p>Example:
 * <pre>{@code
 * CameraConfig front = new CameraConfig.Builder("photon_front")
 *     .robotToCamera(new Transform3d(
 *         new Translation3d(0.3, 0.0, 0.2),
 *         new Rotation3d(0, Math.toRadians(-15), 0)))
 *     .backend(CameraConfig.Backend.PHOTON)
 *     .build();
 * }</pre>
 */
public final class CameraConfig {

  /** Which vision library drives this camera. */
  public enum Backend {
    /** PhotonVision — uses {@code PhotonCamera} and multi-tag PnP. */
    PHOTON,
    /** Limelight — uses YALL; MegaTag 1 and MegaTag 2 supported. */
    LIMELIGHT
  }

  /** Camera or Limelight NT table name as configured in their dashboards. */
  public final String name;

  /**
   * Transform from the robot's origin to the camera lens, in robot coordinates.
   * For Limelights the coprocessor does its own calibration, but this field is
   * still required for documentation and potential single-tag fallback.
   */
  public final Transform3d robotToCamera;

  /** Which library to use for this camera. */
  public final Backend backend;

  /**
   * Multiplier applied to the standard-deviation estimate for this camera.
   * Values {@code > 1} make the estimator trust this camera less;
   * values {@code < 1} trust it more.
   */
  public final double stdDevFactor;

  private CameraConfig(Builder b) {
    this.name = b.name;
    this.robotToCamera = b.robotToCamera;
    this.backend = b.backend;
    this.stdDevFactor = b.stdDevFactor;
  }

  public static final class Builder {
    private final String name;
    private Transform3d robotToCamera = new Transform3d();
    private Backend backend = Backend.PHOTON;
    private double stdDevFactor = 1.0;

    public Builder(String name) {
      if (name == null || name.isBlank()) {
        throw new IllegalArgumentException("Camera name must not be blank");
      }
      this.name = name;
    }

    public Builder robotToCamera(Transform3d t) {
      this.robotToCamera = t;
      return this;
    }

    public Builder backend(Backend b) {
      this.backend = b;
      return this;
    }

    /** Per-camera std dev multiplier (default 1.0). */
    public Builder stdDevFactor(double factor) {
      this.stdDevFactor = factor;
      return this;
    }

    public CameraConfig build() {
      if (robotToCamera == null) throw new IllegalArgumentException("robotToCamera must not be null");
      if (stdDevFactor <= 0) throw new IllegalArgumentException("stdDevFactor must be positive");
      return new CameraConfig(this);
    }
  }
}
