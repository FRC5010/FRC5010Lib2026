package org.frc5010.common.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * {@link VisionIO} implementation backed by a Meta Quest headset running the QuestNav app.
 *
 * <p>Unlike the AprilTag camera backends ({@link VisionIOPhoton}, {@link VisionIOLimelight}),
 * QuestNav is a visual-inertial odometry source: the headset reports a full field-relative
 * pose at 100&nbsp;Hz with no AprilTags involved. Each frame is emitted as a {@link
 * PoseObservationType#QUESTNAV} observation, which {@link Vision} treats specially &mdash; it
 * skips the tag-count / ambiguity rejection (there are no tags) and applies a fixed standard
 * deviation instead of the distance-based model.
 *
 * <h2>Coordinate frames &amp; mounting offset</h2>
 *
 * <p>QuestNav tracks the <em>headset</em>, not the robot. {@link CameraConfig#robotToCamera} is
 * reused here as the robot&nbsp;&rarr;&nbsp;Quest mounting transform:
 *
 * <ul>
 *   <li>To convert a reported Quest pose to a robot pose we apply the inverse offset.
 *   <li>To seed the Quest's field reference frame we apply the forward offset to the robot pose.
 * </ul>
 *
 * <h2>Seeding</h2>
 *
 * <p>QuestNav only reports meaningful field-relative coordinates after {@link QuestNav#setPose}
 * establishes the reference frame. This class seeds it once, the first time the headset is
 * connected and tracking, from the supplied robot pose. Call {@link #requestSeed()} to force a
 * re-seed (e.g. after a known reset to a field wall).
 */
public class VisionIOQuestNav implements VisionIO {

  /** Robot-relative pose used to seed the Quest reference frame (e.g. {@code drive::getPose}). */
  private final Supplier<Pose2d> robotPoseSupplier;

  /** Robot &rarr; Quest mounting transform (from {@link CameraConfig#robotToCamera}). */
  private final Transform3d robotToQuest;

  private final QuestNav questNav = new QuestNav();

  /** True once the Quest field reference frame has been seeded via {@link QuestNav#setPose}. */
  private boolean seeded = false;

  public VisionIOQuestNav(CameraConfig config, Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotToQuest = config.robotToCamera;
  }

  /** Forces the Quest reference frame to be re-seeded from the robot pose on the next cycle. */
  public void requestSeed() {
    seeded = false;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Process command responses (e.g. pose-reset acknowledgements) every loop.
    questNav.commandPeriodic();

    boolean connected = questNav.isConnected();
    inputs.connected = connected;

    // QuestNav has no AprilTag targets — clear the aiming angles.
    inputs.latestTx = Rotation2d.kZero;
    inputs.latestTy = Rotation2d.kZero;
    inputs.tagIds = new int[0];

    // Seed the Quest's field reference frame once it is connected and tracking.
    if (connected && questNav.isTracking() && !seeded) {
      Pose2d robotPose = robotPoseSupplier.get();
      if (robotPose != null) {
        // setPose expects the Quest's pose: robotPose ⊕ (robot → Quest).
        questNav.setPose(new Pose3d(robotPose).transformBy(robotToQuest));
        seeded = true;
      }
    }

    List<Double> timestamps = new ArrayList<>();
    List<Pose3d> poses = new ArrayList<>();

    if (seeded) {
      for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
        if (!frame.isTracking()) {
          continue;
        }
        // Convert the reported Quest pose back to a robot pose: questPose ⊕ (Quest → robot).
        Pose3d robotPose = frame.questPose3d().transformBy(robotToQuest.inverse());
        poses.add(robotPose);
        timestamps.add(frame.dataTimestamp());
      }
    }

    int n = timestamps.size();
    inputs.observationTimestamps = new double[n];
    inputs.observationPoses = new Pose3d[n];
    inputs.observationAmbiguities = new double[n];
    inputs.observationTagCounts = new int[n];
    inputs.observationTagDistances = new double[n];
    inputs.observationTypes = new int[n];
    for (int i = 0; i < n; i++) {
      inputs.observationTimestamps[i] = timestamps.get(i);
      inputs.observationPoses[i] = poses.get(i);
      inputs.observationAmbiguities[i] = 0.0;
      inputs.observationTagCounts[i] = 0;
      inputs.observationTagDistances[i] = 0.0;
      inputs.observationTypes[i] = PoseObservationType.QUESTNAV.ordinal();
    }
  }
}
