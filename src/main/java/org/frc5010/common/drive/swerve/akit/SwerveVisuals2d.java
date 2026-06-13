package org.frc5010.common.drive.swerve.akit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * A {@link Mechanism2d} view of the swerve drivetrain for the regular-simulator
 * dashboards (Glass / Shuffleboard / AdvantageScope's Mechanism tab) — published once
 * to SmartDashboard and updated each cycle by {@link AkitSwerveDrive#periodic()}.
 *
 * <p>Each module is a wheel arrow at its chassis corner: the angle is the live steer
 * azimuth (flipped 180° when the wheel rolls backward) and the length grows with the
 * drive speed, so a stopped wheel is a short stub and a wheel at full speed is a long
 * arrow. A centre needle shows the gyro heading. The robot frame maps to the canvas as
 * forward = up, left = left.
 */
public class SwerveVisuals2d {

  private static final double CANVAS = 4.0;
  private static final double CENTER = CANVAS / 2.0;
  /** Wheel arrow length (canvas units) at zero speed — always visible for orientation. */
  private static final double WHEEL_BASE_LEN = 0.25;
  /** Extra wheel arrow length added at full speed. */
  private static final double WHEEL_SPEED_LEN = 1.0;

  private final double maxSpeedMps;
  private final Mechanism2d canvas;
  private final MechanismLigament2d[] wheels;
  private final MechanismLigament2d gyroNeedle;

  /**
   * Builds and publishes the canvas.
   *
   * @param key                SmartDashboard key (e.g. "SwerveDrive")
   * @param moduleTranslations module positions in the robot frame (x forward, y left)
   * @param maxSpeedMps        drivetrain max linear speed, for normalizing arrow length
   */
  public SwerveVisuals2d(String key, Translation2d[] moduleTranslations, double maxSpeedMps) {
    this.maxSpeedMps = Math.max(1e-6, maxSpeedMps);
    this.canvas = new Mechanism2d(CANVAS, CANVAS);

    double maxAbs = 1e-6;
    for (Translation2d t : moduleTranslations) {
      maxAbs = Math.max(maxAbs, Math.max(Math.abs(t.getX()), Math.abs(t.getY())));
    }
    double layoutScale = (CENTER * 0.6) / maxAbs;

    wheels = new MechanismLigament2d[moduleTranslations.length];
    for (int i = 0; i < moduleTranslations.length; i++) {
      double rootX = CENTER - moduleTranslations[i].getY() * layoutScale; // +Y (left) → canvas left
      double rootY = CENTER + moduleTranslations[i].getX() * layoutScale; // +X (forward) → canvas up
      wheels[i] = canvas.getRoot("Module" + i, rootX, rootY)
          .append(new MechanismLigament2d(
              "wheel" + i, WHEEL_BASE_LEN, 90, 5, new Color8Bit(Color.kAqua)));
    }
    gyroNeedle = canvas.getRoot("Gyro", CENTER, CENTER)
        .append(new MechanismLigament2d(
            "heading", CENTER * 0.7, 90, 4, new Color8Bit(Color.kOrange)));

    SmartDashboard.putData(key, canvas);
  }

  /**
   * Updates the wheel arrows (steer angle + speed-scaled length) and the gyro needle.
   *
   * @param states the measured module states
   * @param gyro   the gyro heading
   */
  public void update(SwerveModuleState[] states, Rotation2d gyro) {
    for (int i = 0; i < wheels.length && i < states.length; i++) {
      double speed = states[i].speedMetersPerSecond;
      double frac = Math.min(1.0, Math.abs(speed) / maxSpeedMps);
      wheels[i].setLength(WHEEL_BASE_LEN + frac * WHEEL_SPEED_LEN);
      // Robot azimuth 0 = forward (+X) = canvas up (90°), CCW positive; flip 180° when
      // the wheel is rolling backward so the arrow points the way the module travels.
      wheels[i].setAngle(90 + states[i].angle.getDegrees() + (speed < 0 ? 180 : 0));
    }
    gyroNeedle.setAngle(90 + gyro.getDegrees());
  }
}
