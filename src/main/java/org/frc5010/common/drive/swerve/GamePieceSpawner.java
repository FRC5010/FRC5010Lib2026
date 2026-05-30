package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

/**
 * Seeds the 2026 Rebuilt arena with a curated set of Fuel game pieces.
 *
 * <p>The default {@code Arena2026Rebuilt.placeGamePiecesOnField()} spawns ≥ 360 pieces
 * (a dense 12×30 center grid plus two depot grids), which is far more physics
 * load than needed for interactive simulation. This spawner clears the default
 * pieces and places 25 Fuel discs at hand-chosen positions that:
 * <ul>
 *   <li>avoid hub footprints ({@code ±0.60 m} from each hub centre),
 *   <li>avoid trench bars and tower walls,
 *   <li>cover all four field quadrants so robots on either alliance have nearby pieces.
 * </ul>
 *
 * <p>Call {@link #spawnInitialFuel()} once after
 * {@link SimulatedArena#addDriveTrainSimulation} from
 * {@link SwerveFactory#build(SwerveConstants, edu.wpi.first.math.geometry.Pose2d)}.
 * It is a no-op on arenas other than {@code Arena2026Rebuilt}.
 *
 * <p>Note: if the user clicks "Reset Field" in the Glass DriverStation NT widget,
 * {@code Arena2026Rebuilt.resetFieldForAuto()} will clear our pieces and call
 * {@code placeGamePiecesOnField()} — restoring the default 360-piece grid.
 * That is expected Glass behaviour and does not require any workaround here.
 */
public final class GamePieceSpawner {

  private GamePieceSpawner() {}

  // 25 curated {x, y} positions in metres (WPILib field frame: X toward Red wall,
  // Y toward Blue driver's left). Chosen to cover all four quadrants while
  // staying ≥ 0.3 m clear of hub footprints, trench bars, towers, and field edges.
  private static final double[][] FUEL_POSITIONS = {
    // ---- centre field ----
    {  6.0,  2.0 }, {  6.0,  4.1 }, {  6.0,  6.2 },
    {  7.5,  1.5 }, {  7.5,  3.2 }, {  7.5,  5.0 }, {  7.5,  6.8 },
    {  9.0,  2.5 }, {  9.0,  4.1 }, {  9.0,  5.7 },
    { 10.5,  1.8 }, { 10.5,  6.5 },
    // ---- blue alliance zone ----
    {  1.5,  2.0 }, {  1.5,  4.0 }, {  1.5,  6.2 },
    {  3.0,  1.5 }, {  3.0,  6.8 },
    // ---- red alliance zone ----
    { 15.0,  2.0 }, { 15.0,  4.0 }, { 15.0,  6.2 },
    { 13.5,  1.5 }, { 13.5,  6.8 },
    // ---- centre line ----
    {  8.27, 1.0 }, {  8.27, 7.2 }, {  8.27, 4.1 },
  };

  /**
   * Clears all existing game pieces from the arena and spawns the curated
   * set of 25 Fuel pieces. No-op when not running {@code Arena2026Rebuilt}.
   */
  public static void spawnInitialFuel() {
    SimulatedArena arena = SimulatedArena.getInstance();
    if (!arena.getClass().getName().contains("Arena2026Rebuilt")) return;

    arena.clearGamePieces();
    for (double[] pos : FUEL_POSITIONS) {
      arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(pos[0], pos[1])));
    }
    System.out.println("[GamePieceSpawner] Spawned " + FUEL_POSITIONS.length
        + " Fuel pieces on Arena2026Rebuilt.");
  }
}
