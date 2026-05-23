package org.frc5010.common.drive.swerve.akit;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import org.frc5010.common.drive.swerve.akit.util.PhoenixUtil;
import swervelib.simulation.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * Physics-based gyro IO backed by a YAGSL IronMaple {@link GyroSimulation}.
 *
 * <p>Used by {@link org.frc5010.common.drive.swerve.SwerveFactory#build} when
 * running in SIM mode with full physics simulation. The {@link GyroSimulation}
 * is driven by the {@link swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation}
 * physics engine — this class only reads from it.
 *
 * <p>This class is intentionally separate from {@link GyroIOSim} so that the
 * {@code instanceof GyroIOSim} check in {@link AkitSwerveDrive} (which drives
 * kinematics-based yaw integration) does not fire for the physics path.
 */
public class GyroIOSimPhysics implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSimPhysics(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
