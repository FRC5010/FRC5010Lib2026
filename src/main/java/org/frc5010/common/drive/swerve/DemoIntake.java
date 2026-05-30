package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 * Demo intake and scoring simulation for the 2026 Rebuilt game.
 * Not a real mechanism — for interactive demonstration only.
 *
 * <p>Web controller button mapping:
 * <ul>
 *   <li>LB (idx 4) — extend and activate intake (stays extended until RB)
 *   <li>RB (idx 5) — retract and stop intake
 *   <li>A  (idx 0) — fire one Fuel at 1 m/s toward the nearest hub
 *   <li>B  (idx 1) — fire one Fuel at 2 m/s
 *   <li>X  (idx 2) — fire one Fuel at 3 m/s
 *   <li>Y  (idx 3) — fire one Fuel at 4 m/s
 * </ul>
 *
 * <p>Intake collision model: while extended the tip is
 * {@code bumperHalf + 12"} (≈ 0.69 m) ahead of the robot centre in the
 * heading direction. Any Fuel piece within {@code INTAKE_RADIUS_M} of that
 * point is removed from the physics world and added to the held count.
 *
 * <p>Firing is demo-only: the held count decrements and the piece is
 * considered scored. A future session can add a RebuiltFuelOnFly projectile
 * arc for visual flair.
 *
 * <p>Must be called from the robot thread (inside the drive default command).
 */
public class DemoIntake {

  // ---- geometry ----
  // Bumper half-length for a nominal 30" square robot: 15" = 0.381 m
  private static final double BUMPER_HALF_M      = Units.inchesToMeters(15);
  private static final double INTAKE_EXTENSION_M = Units.inchesToMeters(12);
  /** Distance from robot centre to intake tip while extended (= bumper edge + 12"). */
  public  static final double INTAKE_REACH_M     = BUMPER_HALF_M + INTAKE_EXTENSION_M;
  private static final double INTAKE_RADIUS_M    = 0.15; // capture radius at tip

  // ---- field constants ----
  private static final Translation2d BLUE_HUB = new Translation2d(4.5974, 4.0345);
  private static final Translation2d RED_HUB  = new Translation2d(11.938, 4.0345);

  private final WebDriveController wdc;
  private boolean intakeExtended = false;
  private int heldFuel = 0;
  private final boolean[] prevBtn = new boolean[6]; // rising-edge detection

  public DemoIntake(WebDriveController wdc) {
    this.wdc = wdc;
  }

  /**
   * Run once per 20 ms robot cycle (inside the drive default command, while enabled).
   * @param robotPose current robot pose from the drive subsystem
   */
  public void periodic(Pose2d robotPose) {
    // ---- intake: LB click latches extended; RB click retracts ----
    if (wdc.getButton(4).getAsBoolean() && !prevBtn[4]) intakeExtended = true;
    if (wdc.getButton(5).getAsBoolean() && !prevBtn[5]) intakeExtended = false;

    if (intakeExtended) collectNearbyFuel(robotPose);

    // ---- fire buttons (rising-edge, A/B/X/Y → 1/2/3/4 m/s) ----
    double[] speeds = { 1.0, 2.0, 3.0, 4.0 };
    for (int i = 0; i < 4; i++) {
      boolean held = wdc.getButton(i).getAsBoolean();
      if (held && !prevBtn[i]) fireFuel(robotPose, speeds[i]);
      prevBtn[i] = held;
    }
    // Update rising-edge state for LB/RB after they've been consumed above.
    prevBtn[4] = wdc.getButton(4).getAsBoolean();
    prevBtn[5] = wdc.getButton(5).getAsBoolean();

    // Push state to WebDriveController atomics so /api/state includes them.
    wdc.setHeldFuel(heldFuel);
    wdc.setIntakeExtended(intakeExtended);
  }

  // ---- private helpers ----

  private void collectNearbyFuel(Pose2d pose) {
    double theta = pose.getRotation().getRadians();
    Translation2d tip = new Translation2d(
        pose.getX() + INTAKE_REACH_M * Math.cos(theta),
        pose.getY() + INTAKE_REACH_M * Math.sin(theta));

    SimulatedArena arena = SimulatedArena.getInstance();
    List<GamePieceOnFieldSimulation> toRemove = new ArrayList<>();
    for (GamePieceOnFieldSimulation piece : arena.gamePiecesOnField()) {
      if (!"Fuel".equals(piece.getType())) continue;
      if (tip.getDistance(piece.getPoseOnField().getTranslation()) < INTAKE_RADIUS_M) {
        toRemove.add(piece);
      }
    }
    for (GamePieceOnFieldSimulation piece : toRemove) {
      if (arena.removeGamePiece(piece)) {
        heldFuel++;
      }
    }
  }

  private void fireFuel(Pose2d pose, double speedMps) {
    if (heldFuel <= 0) return;
    heldFuel--;
    Translation2d robot = pose.getTranslation();
    Translation2d hub = robot.getDistance(BLUE_HUB) < robot.getDistance(RED_HUB)
        ? BLUE_HUB : RED_HUB;
    System.out.printf("[DemoIntake] Fired at %.1f m/s toward hub at (%.2f, %.2f)%n",
        speedMps, hub.getX(), hub.getY());
  }
}
