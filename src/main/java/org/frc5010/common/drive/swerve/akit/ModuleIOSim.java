package org.frc5010.common.drive.swerve.akit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc5010.common.drive.swerve.SwerveConstants;

/** Physics sim implementation of module IO using WPILib DCMotorSim. */
public class ModuleIOSim implements ModuleIO {

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController;
  private final PIDController turnController;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  // Simulation gains — reasonable defaults for a typical FRC swerve drive
  private static final double DRIVE_SIM_KP = 0.05;
  private static final double DRIVE_SIM_KD = 0.0;
  private static final double DRIVE_SIM_KS = 0.0;
  private static final double DRIVE_SIM_KV = 0.0789;
  private static final double TURN_SIM_KP  = 8.0;
  private static final double TURN_SIM_KD  = 0.0;

  // Generic motor models for a Falcon 500-based swerve — simulation accuracy is
  // less important than correct code logic, so exact motor type doesn't matter.
  private static final DCMotor DRIVE_MOTOR    = DCMotor.getFalcon500(1);
  private static final double  DRIVE_REDUCTION = 6.75;   // L2 drive ratio
  private static final DCMotor TURN_MOTOR     = DCMotor.getFalcon500(1);
  private static final double  TURN_REDUCTION  = 150.0 / 7.0; // ~21.4:1 MK4i steer

  public ModuleIOSim(SwerveConstants constants, int index) {
    driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DRIVE_MOTOR, 0.025, DRIVE_REDUCTION),
        DRIVE_MOTOR);
    turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(TURN_MOTOR, 0.004, TURN_REDUCTION),
        TURN_MOTOR);

    driveController = new PIDController(DRIVE_SIM_KP, 0, DRIVE_SIM_KD);
    turnController  = new PIDController(TURN_SIM_KP,  0, TURN_SIM_KD);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // Single odometry sample per 20 ms loop (50 Hz is sufficient for sim)
    inputs.odometryTimestamps = new double[]{Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[]{inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[]{inputs.turnAbsolutePosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_SIM_KS * Math.signum(velocityRadPerSec) + DRIVE_SIM_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
