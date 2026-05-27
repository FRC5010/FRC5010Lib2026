package org.frc5010.common.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.AngularVelocity3d;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * VisionIO implementation for a Limelight camera using the YALL library.
 *
 * <p>MegaTag 2 is read via YALL's {@code LimelightPoseEstimator} (heading-locked, lower noise).
 * MegaTag 1 is read via raw NetworkTables queue subscribers so no frames are dropped between
 * cycles. Robot heading is published each cycle via YALL's {@code withRobotOrientation}.
 *
 * <p>Pass {@code drive::getRotation} as {@code headingSupplier}.
 */
public class VisionIOLimelight implements VisionIO {

  private final Limelight limelight;
  private final LimelightPoseEstimator mt2Estimator;
  private final Supplier<Rotation2d> headingSupplier;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;

  public VisionIOLimelight(CameraConfig config, Supplier<Rotation2d> headingSupplier) {
    this.limelight = new Limelight(config.name);
    this.mt2Estimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
    this.headingSupplier = headingSupplier;

    var table = NetworkTableInstance.getDefault().getTable(config.name);
    latencySubscriber  = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber       = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber       = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber =
        table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Connected if a latency update arrived within 250 ms.
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    inputs.latestTx = Rotation2d.fromDegrees(txSubscriber.get());
    inputs.latestTy = Rotation2d.fromDegrees(tySubscriber.get());

    // Publish heading so the Limelight can lock it for MegaTag 2.
    Rotation3d rot3d = new Rotation3d(0, 0, headingSupplier.get().getRadians());
    AngularVelocity3d zero = new AngularVelocity3d(
        RadiansPerSecond.of(0), RadiansPerSecond.of(0), RadiansPerSecond.of(0));
    limelight.getSettings().withRobotOrientation(new Orientation3d(rot3d, zero));

    Set<Integer> tagIds = new HashSet<>();
    List<Double> timestamps   = new ArrayList<>();
    List<Pose3d> poses        = new ArrayList<>();
    List<Double> ambiguities  = new ArrayList<>();
    List<Integer> tagCounts   = new ArrayList<>();
    List<Double> tagDistances = new ArrayList<>();
    List<Integer> types       = new ArrayList<>();

    // ── MegaTag 1 (raw NT queue — drains all frames since last cycle) ──────
    for (var sample : megatag1Subscriber.readQueue()) {
      if (sample.value.length == 0) continue;
      parseLimelightTagIds(sample.value, tagIds);
      timestamps.add(sample.timestamp * 1.0e-6 - sample.value[6] * 1.0e-3);
      poses.add(parsePose3d(sample.value));
      ambiguities.add(sample.value.length >= 18 ? sample.value[17] : 0.0);
      tagCounts.add((int) sample.value[7]);
      tagDistances.add(sample.value[9]);
      types.add(PoseObservationType.MEGATAG_1.ordinal());
    }

    // ── MegaTag 2 (YALL — heading-locked, ambiguity zeroed) ────────────────
    mt2Estimator.getPoseEstimate().ifPresent(pe -> {
      timestamps.add(pe.timestampSeconds);
      poses.add(pe.pose);
      ambiguities.add(0.0);
      tagCounts.add(pe.tagCount);
      tagDistances.add(pe.avgTagDist);
      types.add(PoseObservationType.MEGATAG_2.ordinal());
    });

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

  // ── Helpers ─────────────────────────────────────────────────────────────────

  private static Pose3d parsePose3d(double[] v) {
    return new Pose3d(
        new edu.wpi.first.math.geometry.Translation3d(v[0], v[1], v[2]),
        new Rotation3d(
            edu.wpi.first.math.util.Units.degreesToRadians(v[3]),
            edu.wpi.first.math.util.Units.degreesToRadians(v[4]),
            edu.wpi.first.math.util.Units.degreesToRadians(v[5])));
  }

  private static void parseLimelightTagIds(double[] v, Set<Integer> tagIds) {
    for (int i = 11; i < v.length; i += 7) {
      tagIds.add((int) v[i]);
    }
  }
}
