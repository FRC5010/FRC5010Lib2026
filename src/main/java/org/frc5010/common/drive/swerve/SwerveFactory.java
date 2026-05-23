package org.frc5010.common.drive.swerve;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.akit.GyroIO;
import org.frc5010.common.drive.swerve.akit.GyroIONavX;
import org.frc5010.common.drive.swerve.akit.GyroIOPigeon2;
import org.frc5010.common.drive.swerve.akit.GyroIOSim;
import org.frc5010.common.drive.swerve.akit.GyroIOSimPhysics;
import org.frc5010.common.drive.swerve.akit.ModuleIO;
import org.frc5010.common.drive.swerve.akit.ModuleIOSim;
import org.frc5010.common.drive.swerve.akit.ModuleIOSimPhysics;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/**
 * Factory that constructs a fully wired {@link AkitSwerveDrive} from a
 * {@link SwerveConstants} configuration record.
 *
 * <p>Two factory methods are available:
 * <ul>
 *   <li>{@link #build} — full IronMaple physics simulation in SIM mode (layers 2 & 3).
 *       Real hardware and replay are unchanged.
 *   <li>{@link #buildWithoutPhysics} — lightweight WPILib {@code DCMotorSim} in SIM mode
 *       (layer 1 unit tests, no YAGSL physics overhead).
 * </ul>
 *
 * <p>Usage in RobotContainer:
 * <pre>{@code
 * AkitSwerveDrive drive = SwerveFactory.build(Constants.SWERVE);
 * }</pre>
 */
public class SwerveFactory {

  private SwerveFactory() {}

  // ---------------------------------------------------------------------------
  // Public API
  // ---------------------------------------------------------------------------

  /**
   * Builds an {@link AkitSwerveDrive} with full IronMaple physics simulation in SIM mode.
   *
   * <p>In SIM mode the physics engine ({@link SwerveDriveSimulation}) drives the robot's motion.
   * {@link AkitSwerveDrive#simulationPeriodic()} automatically advances the arena each loop.
   * Default COTS hardware (Mark4 L2 with Falcon500, Colsons) is used; the track
   * width/wheelbase come from {@code constants}.
   *
   * <p>In REAL and REPLAY modes the behavior is identical to {@link #buildWithoutPhysics}.
   *
   * @param constants  the robot's swerve configuration
   * @return a fully constructed and wired drivetrain subsystem
   */
  public static AkitSwerveDrive build(SwerveConstants constants) {
    Mode mode = RobotMode.get();

    if (mode == Mode.SIM) {
      return buildWithPhysicsSim(constants);
    }

    GyroIO gyro = buildGyro(constants, mode);
    ModuleIO[] modules = buildModules(constants, mode);
    return new AkitSwerveDrive(constants, gyro, modules);
  }

  /**
   * Builds an {@link AkitSwerveDrive} using lightweight WPILib {@code DCMotorSim} in SIM mode.
   *
   * <p>Suitable for layer-1 unit tests where YAGSL physics overhead is undesirable.
   * In REAL and REPLAY modes this is identical to {@link #build}.
   *
   * @param constants  the robot's swerve configuration
   * @return a fully constructed and wired drivetrain subsystem
   */
  public static AkitSwerveDrive buildWithoutPhysics(SwerveConstants constants) {
    Mode mode = RobotMode.get();
    GyroIO gyro = buildGyro(constants, mode);
    ModuleIO[] modules = buildModules(constants, mode);
    return new AkitSwerveDrive(constants, gyro, modules);
  }

  // ---------------------------------------------------------------------------
  // Physics-enabled SIM build
  // ---------------------------------------------------------------------------

  private static AkitSwerveDrive buildWithPhysicsSim(SwerveConstants c) {
    DriveTrainSimulationConfig simConfig =
        DriveTrainSimulationConfig.Default()
            .withTrackLengthTrackWidth(
                Meters.of(c.wheelBaseMeters), Meters.of(c.trackWidthMeters));

    SwerveDriveSimulation swerveDriveSim =
        new SwerveDriveSimulation(simConfig, Pose2d.kZero);
    SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSim);

    GyroIO gyro = new GyroIOSimPhysics(swerveDriveSim.getGyroSimulation());

    swervelib.simulation.ironmaple.simulation.drivesims.SwerveModuleSimulation[] moduleSims =
        swerveDriveSim.getModules();
    ModuleIO[] modules = new ModuleIO[] {
      new ModuleIOSimPhysics(moduleSims[0]),
      new ModuleIOSimPhysics(moduleSims[1]),
      new ModuleIOSimPhysics(moduleSims[2]),
      new ModuleIOSimPhysics(moduleSims[3]),
    };

    return new AkitSwerveDrive(c, gyro, modules, swerveDriveSim);
  }

  // ---------------------------------------------------------------------------
  // Module IO selection (non-physics paths)
  // ---------------------------------------------------------------------------

  private static ModuleIO[] buildModules(SwerveConstants c, Mode mode) {
    return switch (mode) {
      case REAL   -> buildRealModules(c);
      case SIM    -> buildSimModules(c);
      case REPLAY -> buildReplayModules();
    };
  }

  private static ModuleIO[] buildRealModules(SwerveConstants c) {
    switch (c.moduleType) {
      case TALON_FX:
        throw new UnsupportedOperationException(
            "ModuleType.TALON_FX requires SwerveModuleConstants generated by CTRE TunerX. "
            + "Instantiate ModuleIOTalonFXReal(swerveConfig, moduleConstants) directly in your "
            + "robot project rather than relying on SwerveFactory for REAL mode.");
      case SPARK_TALON:
        throw new UnsupportedOperationException(
            "ModuleType.SPARK_TALON requires SwerveModuleConstants generated by CTRE TunerX. "
            + "Instantiate ModuleIOSparkTalon(swerveConfig, moduleConstants) directly in your "
            + "robot project rather than relying on SwerveFactory for REAL mode.");
      case SPARK_MAX:
        throw new IllegalArgumentException(
            "ModuleType.SPARK_MAX is not yet implemented. Use TALON_FX or SPARK_TALON.");
      case SIM:
        throw new IllegalArgumentException(
            "ModuleType.SIM cannot be used in REAL mode. "
            + "Set RobotMode to SIM or choose a hardware module type.");
      default:
        throw new IllegalArgumentException("Unknown module type: " + c.moduleType);
    }
  }

  private static ModuleIO[] buildSimModules(SwerveConstants c) {
    return new ModuleIO[] {
      new ModuleIOSim(c, 0),
      new ModuleIOSim(c, 1),
      new ModuleIOSim(c, 2),
      new ModuleIOSim(c, 3),
    };
  }

  private static ModuleIO[] buildReplayModules() {
    return new ModuleIO[] {
      new ModuleIO() {},
      new ModuleIO() {},
      new ModuleIO() {},
      new ModuleIO() {},
    };
  }

  // ---------------------------------------------------------------------------
  // Gyro IO selection
  // ---------------------------------------------------------------------------

  private static GyroIO buildGyro(SwerveConstants c, Mode mode) {
    return switch (mode) {
      case REAL   -> buildRealGyro(c);
      case SIM    -> new GyroIOSim();
      case REPLAY -> new GyroIO() {};
    };
  }

  private static GyroIO buildRealGyro(SwerveConstants c) {
    return switch (c.gyroType) {
      case PIGEON2 -> new GyroIOPigeon2(c);
      case NAVX    -> new GyroIONavX();
      case SIM     -> throw new IllegalArgumentException(
          "GyroType.SIM cannot be used in REAL mode. " +
          "Set RobotMode to SIM or choose a hardware gyro type.");
    };
  }
}
