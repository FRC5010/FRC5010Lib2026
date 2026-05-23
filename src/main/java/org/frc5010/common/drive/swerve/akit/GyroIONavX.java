package org.frc5010.common.drive.swerve.akit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * NavX gyro IO implementation.
 *
 * <p><b>Requires the Kauai Labs NavX vendordep.</b> Add the following to your vendordeps directory
 * before using this class in REAL mode:
 * <pre>https://dev.studica.com/releases/2025/NavX.json</pre>
 *
 * <p>This stub compiles without the vendordep so the framework builds cleanly. When the
 * vendordep is present, replace this class with a full AHRS-backed implementation.
 */
public class GyroIONavX implements GyroIO {

  public GyroIONavX() {
    // Full implementation requires the Kauai Labs NavX vendordep.
    // See class-level Javadoc for the vendordep URL.
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = false;
    inputs.yawPosition = new Rotation2d();
    inputs.yawVelocityRadPerSec = 0.0;
    inputs.odometryYawTimestamps = new double[]{Timer.getFPGATimestamp()};
    inputs.odometryYawPositions = new Rotation2d[]{new Rotation2d()};
  }
}
