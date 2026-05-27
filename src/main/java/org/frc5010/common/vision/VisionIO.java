package org.frc5010.common.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  class VisionIOInputs {
    public boolean connected = false;

    // Raw tx/ty to the best target — useful for simple servoing.
    public Rotation2d latestTx = Rotation2d.kZero;
    public Rotation2d latestTy = Rotation2d.kZero;

    // Parallel arrays — one element per pose observation this cycle.
    // Pose3d implements WPILib Struct so AdvantageKit can serialize it.
    public double[] observationTimestamps   = new double[0];
    public Pose3d[] observationPoses        = new Pose3d[0];
    public double[] observationAmbiguities  = new double[0];
    public int[]    observationTagCounts    = new int[0];
    public double[] observationTagDistances = new double[0];
    /** Ordinal of {@link PoseObservationType} for each observation. */
    public int[]    observationTypes        = new int[0];

    /** All AprilTag IDs seen this cycle. */
    public int[] tagIds = new int[0];
  }

  /** Raw angle to a target — not used for pose estimation but useful for aiming. */
  record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Reconstructed in Vision.periodic() from the parallel arrays above. */
  record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
