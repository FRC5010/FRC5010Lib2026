package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/**
 * VisionIO implementation for a real PhotonVision camera.
 *
 * <p>Uses {@code camera.getAllUnreadResults()} to consume every pipeline result since the
 * last call, converting multi-tag PnP results and single-tag results into parallel arrays
 * stored in {@link VisionIOInputs}.  No deprecated strategy enum is used.
 */
public class VisionIOPhoton implements VisionIO {

  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final AprilTagFieldLayout layout;

  public VisionIOPhoton(CameraConfig config, AprilTagFieldLayout layout) {
    this.camera = new PhotonCamera(config.name);
    this.robotToCamera = config.robotToCamera;
    this.layout = layout;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    List<Double> timestamps   = new ArrayList<>();
    List<Pose3d> poses        = new ArrayList<>();
    List<Double> ambiguities  = new ArrayList<>();
    List<Integer> tagCounts   = new ArrayList<>();
    List<Double> tagDistances = new ArrayList<>();
    List<Integer> types       = new ArrayList<>();

    for (var result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        inputs.latestTx = Rotation2d.fromDegrees(result.getBestTarget().getYaw());
        inputs.latestTy = Rotation2d.fromDegrees(result.getBestTarget().getPitch());
      } else {
        inputs.latestTx = Rotation2d.kZero;
        inputs.latestTy = Rotation2d.kZero;
      }

      if (result.multitagResult.isPresent()) {
        // ── Multi-tag PnP (coprocessor) ────────────────────────────────────
        var mt = result.multitagResult.get();
        Transform3d fieldToCamera = mt.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        double totalDist = 0.0;
        for (var t : result.targets) {
          totalDist += t.bestCameraToTarget.getTranslation().getNorm();
        }

        tagIds.addAll(mt.fiducialIDsUsed);
        timestamps.add(result.getTimestampSeconds());
        poses.add(robotPose);
        ambiguities.add(mt.estimatedPose.ambiguity);
        tagCounts.add(mt.fiducialIDsUsed.size());
        tagDistances.add(totalDist / result.targets.size());
        types.add(PoseObservationType.PHOTONVISION.ordinal());

      } else if (!result.targets.isEmpty()) {
        // ── Single-tag fallback ─────────────────────────────────────────────
        var target = result.targets.get(0);
        var tagPose = layout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d fieldToCamera = fieldToTarget.plus(target.bestCameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          tagIds.add((short) target.fiducialId);
          timestamps.add(result.getTimestampSeconds());
          poses.add(robotPose);
          ambiguities.add(target.poseAmbiguity);
          tagCounts.add(1);
          tagDistances.add(target.bestCameraToTarget.getTranslation().getNorm());
          types.add(PoseObservationType.PHOTONVISION.ordinal());
        }
      }
    }

    int n = timestamps.size();
    inputs.observationTimestamps   = new double[n];
    inputs.observationPoses        = new Pose3d[n];
    inputs.observationAmbiguities  = new double[n];
    inputs.observationTagCounts    = new int[n];
    inputs.observationTagDistances = new double[n];
    inputs.observationTypes        = new int[n];
    for (int i = 0; i < n; i++) {
      inputs.observationTimestamps[i]   = timestamps.get(i);
      inputs.observationPoses[i]        = poses.get(i);
      inputs.observationAmbiguities[i]  = ambiguities.get(i);
      inputs.observationTagCounts[i]    = tagCounts.get(i);
      inputs.observationTagDistances[i] = tagDistances.get(i);
      inputs.observationTypes[i]        = types.get(i);
    }

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) inputs.tagIds[i++] = id;
  }
}
