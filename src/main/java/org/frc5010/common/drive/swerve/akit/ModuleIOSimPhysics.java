package org.frc5010.common.drive.swerve.akit;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import org.frc5010.common.drive.swerve.akit.util.PhoenixUtil;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Generic physics-based module IO backed by a YAGSL IronMaple {@link SwerveModuleSimulation}.
 *
 * <p>Used by {@link org.frc5010.common.drive.swerve.SwerveFactory#build} for the full-physics
 * simulation path. Unlike {@link ModuleIOTalonFXSim} (which wires real TalonFX hardware objects
 * into the physics engine), this class uses generic voltage controllers so it works without any
 * TunerX-generated {@code SwerveModuleConstants}.
 *
 * <p>Drive velocity is controlled via back-EMF feedforward; steer position uses a proportional
 * voltage controller. Both are sufficient for realistic physics simulation even if not as
 * accurate as the TalonFX control loops on real hardware.
 */
public class ModuleIOSimPhysics implements ModuleIO {
  private static final double STEER_KP = 8.0;

  private final SwerveModuleSimulation moduleSim;
  private final SimulatedMotorController.GenericMotorController driveController;
  private final SimulatedMotorController.GenericMotorController steerController;

  public ModuleIOSimPhysics(SwerveModuleSimulation moduleSim) {
    this.moduleSim = moduleSim;
    this.driveController = moduleSim.useGenericMotorControllerForDrive();
    this.steerController = moduleSim.useGenericControllerForSteer();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSim.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = moduleSim.getDriveMotorAppliedVoltage().in(Volts);
    inputs.driveCurrentAmps = moduleSim.getDriveMotorStatorCurrent().in(Amps);

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = moduleSim.getSteerAbsoluteFacing();
    inputs.turnPosition = moduleSim.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec = moduleSim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = moduleSim.getSteerMotorAppliedVoltage().in(Volts);
    inputs.turnCurrentAmps = moduleSim.getSteerMotorStatorCurrent().in(Amps);

    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions())
            .mapToDouble(a -> a.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = moduleSim.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveController.requestVoltage(Volts.of(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double motorRadPerSec = velocityRadPerSec * moduleSim.config.DRIVE_GEAR_RATIO;
    double voltage =
        motorRadPerSec / moduleSim.getDriveMotorConfigs().motor.KvRadPerSecPerVolt;
    driveController.requestVoltage(Volts.of(MathUtil.clamp(voltage, -12.0, 12.0)));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double error =
        MathUtil.angleModulus(rotation.minus(moduleSim.getSteerAbsoluteFacing()).getRadians());
    double voltage = STEER_KP * error;
    steerController.requestVoltage(Volts.of(MathUtil.clamp(voltage, -12.0, 12.0)));
  }
}
