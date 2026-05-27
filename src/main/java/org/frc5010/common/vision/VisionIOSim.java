package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * VisionIO simulation implementation using PhotonVision's {@link VisionSystemSim}.
 *
 * <p>Extends {@link VisionIOPhoton} — the same {@code PhotonCamera} object is used for both
 * real and simulated result parsing, so {@code updateInputs()} logic is shared.
 * Each cycle, the sim is updated with the robot's current pose before reading results.
 *
 * <p>Works for both Layer 2 tests ({@code buildWithoutPhysics}) and Layer 3 tests
 * ({@code build()}) — the caller supplies the pose via {@code poseSupplier}.
 */
public class VisionIOSim extends VisionIOPhoton {

  private final VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;

  /**
   * @param config       Camera config — name, transform, and backend (must be PHOTON).
   * @param layout       Field AprilTag layout used to place simulated targets.
   * @param poseSupplier Current robot pose; typically {@code drive::getPose}.
   */
  public VisionIOSim(CameraConfig config, AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier) {
    super(config, layout); // creates the PhotonCamera
    this.poseSupplier = poseSupplier;

    visionSim = new VisionSystemSim("vision_" + config.name);
    visionSim.addAprilTags(layout);

    SimCameraProperties props = SimCameraProperties.PERFECT_90DEG();
    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, props);
    cameraSim.enableDrawWireframe(true);
    visionSim.addCamera(cameraSim, config.robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get()); // advance sim to current robot pose
    super.updateInputs(inputs);           // read camera results via PhotonCamera
  }
}
