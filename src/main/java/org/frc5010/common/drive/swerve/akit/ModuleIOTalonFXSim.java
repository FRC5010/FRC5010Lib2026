// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.frc5010.common.drive.swerve.akit;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import java.util.Arrays;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.akit.util.PhoenixUtil;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Physics sim implementation of module IO for TalonFX-based swerve modules.
 *
 * <p>Extends {@link ModuleIOTalonFX} to wire the real Phoenix hardware state
 * (TalonFX + CANcoder sim states) into the YAGSL IronMaple physics engine.
 * Use this when running robot code in simulation with full TunerX-generated
 * {@link SwerveModuleConstants}.
 *
 * <p>Instantiate directly in your robot project for the physics simulation path:
 * <pre>{@code
 * SwerveDriveSimulation driveSim = ...;
 * SwerveModuleSimulation[] moduleSims = driveSim.getModules();
 * ModuleIO fl = new ModuleIOTalonFXSim(swerveConstants, flModuleConstants, moduleSims[0]);
 * }</pre>
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  public ModuleIOTalonFXSim(
      SwerveConstants swerveConfig,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      SwerveModuleSimulation simulation) {
    super(swerveConfig, PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.simulation = simulation;
    simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));
    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }
}
