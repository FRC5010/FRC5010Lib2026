// FRC5010 Framework — clean WPILib-native gyro simulation

package org.frc5010.common.drive.swerve.akit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simulation implementation of {@link GyroIO} using pure WPILib math.
 *
 * <p>Rather than depending on YAGSL's IronMaple {@code GyroSimulation},
 * this implementation integrates chassis angular velocity supplied from
 * the drivetrain each cycle. The drivetrain calls
 * {@link #updateAngularVelocity(double)} once per loop with the yaw rate
 * derived from kinematics, and this class integrates it into a simulated
 * heading.
 *
 * <p>This approach keeps the gyro sim dependency-free and testable without
 * any third-party simulation framework.
 */
public class GyroIOSim implements GyroIO {

  private Rotation2d yawPosition = new Rotation2d();
  private double yawVelocityRadPerSec = 0.0;
  private double lastTimestamp = Timer.getFPGATimestamp();

  /**
   * Called by {@link AkitSwerveDrive} each periodic loop with the angular
   * velocity derived from module kinematics.
   *
   * @param angularVelocityRadPerSec robot yaw rate in radians per second,
   *                                  positive = counter-clockwise
   */
  public void updateAngularVelocity(double angularVelocityRadPerSec) {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;

    yawVelocityRadPerSec = angularVelocityRadPerSec;
    yawPosition = yawPosition.plus(
        Rotation2d.fromRadians(angularVelocityRadPerSec * dt));
  }

  /** Resets the simulated heading to a known pose. Used when pose is reset. */
  public void resetYaw(Rotation2d newYaw) {
    yawPosition = newYaw;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = yawPosition;
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;

    // Single-sample odometry in sim (50 Hz is sufficient)
    inputs.odometryYawTimestamps = new double[]{Timer.getFPGATimestamp()};
    inputs.odometryYawPositions = new Rotation2d[]{yawPosition};
  }
}
