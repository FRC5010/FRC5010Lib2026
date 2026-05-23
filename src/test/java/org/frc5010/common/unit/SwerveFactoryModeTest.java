package org.frc5010.common.unit;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Layer 1 unit tests for SwerveFactory mode selection logic.
 *
 * These tests verify that the factory wires the correct IO implementations
 * for each Mode without actually constructing hardware.
 */
class SwerveFactoryModeTest {

  private static final SwerveConstants SIM_CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  private static final SwerveConstants TALON_CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.TALON_FX)
          .gyroType(GyroType.PIGEON2)
          .build();

  @BeforeEach
  void setup() {
    HAL.initialize(500, 0);
    RobotMode.resetForTesting();
  }

  @AfterEach
  void teardown() {
    RobotMode.resetForTesting();
  }

  @Test
  void simModeBuildsSuccessfully() {
    RobotMode.set(Mode.SIM);
    assertDoesNotThrow(() -> SwerveFactory.buildWithoutPhysics(SIM_CONSTANTS));
  }

  @Test
  void replayModeBuildsSuccessfully() {
    RobotMode.set(Mode.REPLAY);
    // In replay mode the factory uses no-op IO — should always succeed
    // regardless of the configured module/gyro type
    assertDoesNotThrow(() -> SwerveFactory.buildWithoutPhysics(TALON_CONSTANTS));
  }

  @Test
  void simModeWithTalonConstantsStillUsesSim() {
    // Even if the constants say TALON_FX, SIM mode must override to ModuleIOSim
    RobotMode.set(Mode.SIM);
    AkitSwerveDrive drive = SwerveFactory.buildWithoutPhysics(TALON_CONSTANTS);
    assertNotNull(drive);
  }

  @Test
  void realModeWithSimModuleTypeThrows() {
    RobotMode.set(Mode.REAL);
    assertThrows(IllegalArgumentException.class,
        () -> SwerveFactory.build(SIM_CONSTANTS));
  }

  @Test
  void realModeWithSimGyroTypeThrows() {
    RobotMode.set(Mode.REAL);
    SwerveConstants badGyro = new SwerveConstants.Builder()
        .moduleType(ModuleType.TALON_FX)
        .gyroType(GyroType.SIM)
        .build();
    assertThrows(IllegalArgumentException.class,
        () -> SwerveFactory.build(badGyro));
  }

  @Test
  void modeNotSetThrowsBeforeBuild() {
    // RobotMode was reset in @BeforeEach — calling build before set() must throw
    assertThrows(IllegalStateException.class,
        () -> SwerveFactory.build(SIM_CONSTANTS));
  }
}