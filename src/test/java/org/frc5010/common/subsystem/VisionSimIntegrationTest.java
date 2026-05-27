package org.frc5010.common.subsystem;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.frc5010.common.vision.CameraConfig;
import org.frc5010.common.vision.Vision;
import org.frc5010.common.vision.VisionFactory;
import org.frc5010.common.vision.VisionIO.VisionIOInputs;
import org.frc5010.common.vision.VisionIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Layer 3 — integration tests combining IronMaple physics ({@link SwerveFactory#build}) with
 * {@link VisionIOSim} (PhotonVision camera simulation).
 *
 * <p>All tests run headlessly: {@code VisionSystemSim} is pure math (ray-casting + tag
 * projection); it needs no display window and is safe in CI.
 *
 * <h3>Camera geometry</h3>
 * <p>Front camera: 30 cm forward, 50 cm up, no rotation (faces robot +X). From the spawn pose
 * (2.0 m, 4.0 m, 0°), the camera is at (2.3 m, 4.0 m) facing +X. 2026 Rebuilt Welded tags 25
 * and 26 sit at x ≈ 4.02 m, y ≈ 4.04–4.39 m with yaw = 180° (face toward −X / toward the
 * camera). Distance ≈ 1.7 m, lateral angles 1°–13° — confirmed well within the 90° FOV of
 * {@code SimCameraProperties.PERFECT_90DEG()}.
 *
 * <h3>Per-cycle order</h3>
 * <pre>
 *   drive.simulationPeriodic()  // advance IronMaple dyn4j world
 *   drive.periodic()            // read physics state → pose estimator
 *   vision.periodic()           // update camera sim → consumer
 *   stepOneCycle()              // advance FPGA clock 20 ms
 * </pre>
 */
class VisionSimIntegrationTest extends SimTestBase {

  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  /**
   * Robot spawn that places the front camera in direct line-of-sight of tags 25 and 26.
   * Heading 0° → camera faces +X toward the Blue Reef (x ≈ 4 m).
   */
  private static final Pose2d VISION_SPAWN = new Pose2d(2.0, 4.0, Rotation2d.kZero);

  // Front-facing camera: 30 cm forward, 50 cm up, optical axis = robot +X.
  private static final Transform3d FRONT_CAM = new Transform3d(
      new Translation3d(0.30, 0.0, 0.50), new Rotation3d());

  private static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private AkitSwerveDrive drive;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.build(CONSTANTS, VISION_SPAWN);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    SimulatedArena.getInstance().shutDown();
    try {
      java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("Failed to reset SimulatedArena singleton", e);
    }
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  // ---------------------------------------------------------------------------
  // Helper: one physics + vision cycle
  // ---------------------------------------------------------------------------

  private void step(Vision vision) {
    drive.simulationPeriodic();
    drive.periodic();
    vision.periodic();
    stepOneCycle();
  }

  // ---------------------------------------------------------------------------
  // Test 1 — VisionIOSim detects tags headlessly (no drive, no scheduler)
  // ---------------------------------------------------------------------------

  /**
   * Verifies that {@link VisionIOSim} generates observations when the robot is placed at
   * {@link #VISION_SPAWN}. This test does NOT use the full {@link Vision} subsystem — it calls
   * {@code updateInputs} directly so it can inspect the raw camera output.
   *
   * <p>Tags 25 and 26 (Blue Reef, x ≈ 4.02 m, yaw = 180°) are within the camera's 90° FOV
   * at ≈ 1.7 m. {@code PERFECT_90DEG} has no noise and no range limit, so detection is
   * guaranteed if the geometry is correct.
   */
  @Test
  void visionIOSimDetectsTagsAtKnownPoseHeadless() {
    CameraConfig cfg = new CameraConfig.Builder("photon_front")
        .robotToCamera(FRONT_CAM)
        .backend(CameraConfig.Backend.PHOTON)
        .build();

    VisionIOSim io = new VisionIOSim(cfg, LAYOUT, () -> VISION_SPAWN);

    VisionIOInputs inputs = new VisionIOInputs();
    io.updateInputs(inputs);

    assertTrue(inputs.observationTimestamps.length >= 1,
        "VisionIOSim must detect at least one observation (tags 25 and 26 expected) "
            + "from (2,4) heading=0° — check camera geometry or field layout");
  }

  // ---------------------------------------------------------------------------
  // Test 2 — full pipeline: physics drive + VisionIOSim → consumer called
  // ---------------------------------------------------------------------------

  /**
   * Verifies the complete Layer 3 pipeline: IronMaple physics drive + camera sim +
   * {@link Vision} subsystem forwarding accepted observations to the consumer.
   *
   * <p>The robot is held stationary at {@link #VISION_SPAWN}. After 20 cycles (400 ms sim
   * time), the consumer must have been called at least once, confirming that
   * {@code VisionIOSim} detects tags and {@code Vision.periodic()} accepts and forwards them.
   */
  @Test
  void visionConsumerCalledDuringPhysicsSimulation() {
    List<Pose2d> received = new ArrayList<>();

    Vision vision = VisionFactory.build(
        (pose, ts, s) -> received.add(pose),
        () -> drive.getSimulatedPose().orElse(drive.getPose()),
        drive::getRotation,
        new CameraConfig[] {
            new CameraConfig.Builder("photon_front2")
                .robotToCamera(FRONT_CAM)
                .backend(CameraConfig.Backend.PHOTON)
                .build()
        });

    for (int i = 0; i < 20; i++) step(vision);

    assertFalse(received.isEmpty(),
        "Vision consumer must be called at least once in 20 cycles; "
            + "check that VisionIOSim sees tags at VISION_SPAWN=(2,4,0°)");
  }

  // ---------------------------------------------------------------------------
  // Test 3 — camera sim uses physics pose, not estimator
  // ---------------------------------------------------------------------------

  /**
   * Verifies that {@link VisionIOSim} tracks the TRUE physics body position, not the
   * pose estimator, so that injected estimator errors do not blind the camera.
   *
   * <p>The physics body is at {@link #VISION_SPAWN} (2 m, 4 m). After one cycle that
   * establishes the physics position, the estimator is shifted 3 m in X via
   * {@code drive.setPose(5, 4)}. Then vision runs for 5 more cycles. Because
   * {@code poseSupplier = () -> drive.getSimulatedPose().orElse(drive.getPose())} always
   * returns the physics truth, the camera should still see the Blue Reef tags and the
   * consumer should continue to be called — proving the supplier does NOT follow the
   * corrupt estimator.
   */
  @Test
  void visionSimUsesPhysicsPoseNotEstimator() {
    List<Pose2d> received = new ArrayList<>();

    Vision vision = VisionFactory.build(
        (pose, ts, s) -> received.add(pose),
        () -> drive.getSimulatedPose().orElse(drive.getPose()),
        drive::getRotation,
        new CameraConfig[] {
            new CameraConfig.Builder("photon_front3")
                .robotToCamera(FRONT_CAM)
                .backend(CameraConfig.Backend.PHOTON)
                .build()
        });

    // One warm-up cycle so physics body is settled at VISION_SPAWN.
    step(vision);
    received.clear();

    // Inject a 3 m X error into the estimator only; physics body stays at (2, 4).
    drive.setPose(new Pose2d(5.0, 4.0, Rotation2d.kZero));

    // Run 5 more cycles.  Camera sim must still use physics pose → still sees tags.
    for (int i = 0; i < 5; i++) step(vision);

    assertFalse(received.isEmpty(),
        "Vision consumer must still be called after estimator error injection — "
            + "VisionIOSim must use the physics body pose (getSimulatedPose), not drive.getPose()");
  }
}
