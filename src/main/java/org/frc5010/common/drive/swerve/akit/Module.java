// Copyright (c) 2021-2025 Littleton Robotics (adapted for FRC5010 framework)
// Use of this source code is governed by a BSD license.

package org.frc5010.common.drive.swerve.akit;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a single swerve module, bridging the ModuleIO hardware abstraction
 * to the higher-level drivetrain logic.
 *
 * <p>This version removes all dependencies on {@code frc.robot.Robot},
 * {@code AkitSwerveConfig}, and CTRE {@code SwerveModuleConstants}. The only
 * robot-specific value needed at this level is the wheel radius, passed in at
 * construction from {@link org.frc5010.common.drive.swerve.SwerveConstants}.
 */
public class Module {

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final double wheelRadiusMeters;

  // Module location used for characterization rotation targeting
  private final double locationX;
  private final double locationY;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;

  /** Grow-only array: never shrunk to minimise GC pressure from per-cycle allocation. */
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[0];

  /**
   * Creates a new Module.
   *
   * @param io               the hardware IO implementation for this module
   * @param index            module index (0=FL, 1=FR, 2=BL, 3=BR) used for logging
   * @param wheelRadiusMeters physical wheel radius in meters from SwerveConstants
   * @param locationX        X position of this module relative to robot center (meters)
   * @param locationY        Y position of this module relative to robot center (meters)
   */
  public Module(ModuleIO io, int index, double wheelRadiusMeters,
      double locationX, double locationY) {
    this.io = io;
    this.index = index;
    this.wheelRadiusMeters = wheelRadiusMeters;
    this.locationX = locationX;
    this.locationY = locationY;

    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert("Disconnected turn encoder on module " + index + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    // Calculate positions for odometry
    int sampleCount =
        Math.min(
            inputs.odometryDrivePositionsRad.length,
            inputs.odometryTurnPositions.length);

    // Grow the array only when needed; never shrink to avoid per-cycle GC pressure
    if (odometryPositions.length != sampleCount) {
      odometryPositions = new SwerveModulePosition[sampleCount];
      for (int i = 0; i < sampleCount; i++) {
        odometryPositions[i] = new SwerveModulePosition();
      }
    }
    for (int i = 0; i < sampleCount; i++) {
      odometryPositions[i].distanceMeters =
          inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
      odometryPositions[i].angle = inputs.odometryTurnPositions[i];
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /**
   * Runs the module with the specified setpoint state. Mutates the state to optimize it.
   *
   * <p>Uses absolute position in simulation (where turnPosition is always zero
   * from the encoder) and relative position on real hardware.
   */
  public void runSetpoint(SwerveModuleState state, Current torqueCurrent) {
    state.optimize(getAngle());
    state.cosineScale(RobotBase.isSimulation()
        ? inputs.turnAbsolutePosition
        : inputs.turnPosition);

    io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters, torqueCurrent);
    io.setTurnPosition(state.angle);
  }

  public void runSetpoint(SwerveModuleState state) {
    runSetpoint(state, Amps.zero());
  }

  /**
   * Runs drive open-loop for characterization while holding the module at the
   * correct rotation angle for full-robot spin characterization.
   */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(
        new Rotation2d(locationX, locationY).plus(Rotation2d.kCCW_Pi_2));
  }

  /** Runs steer open-loop for steer characterization while holding drive at zero. */
  public void runSteerCharacterization(double output) {
    io.setDriveOpenLoop(0);
    io.setTurnOpenLoop(output);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /**
   * Returns the current turn angle. Uses absolute position in sim,
   * relative encoder position on real hardware.
   */
  public Rotation2d getAngle() {
    return RobotBase.isSimulation()
        ? inputs.turnAbsolutePosition
        : inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the current drive acceleration state for chassis kinematics. */
  public SwerveModuleState getAccelerationState() {
    return new SwerveModuleState(
        inputs.driveAccelerationRadPerSecSquared * wheelRadiusMeters,
        getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the odometry samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the drive position in radians (for wheel radius characterization). */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the drive velocity in rad/sec (for feedforward characterization). */
  public double getDriveFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  /** Returns the steer velocity in rad/sec (for steer feedforward characterization). */
  public double getSteerFFCharacterizationVelocity() {
    return inputs.turnVelocityRadPerSec;
  }
}
