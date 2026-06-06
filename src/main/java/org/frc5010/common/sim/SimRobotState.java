package org.frc5010.common.sim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Abstract base for sim-mode game-piece subsystems.
 *
 * <p>Owns the {@link IntakeSimulation} instance — registers it at construction,
 * cleans up obtained game pieces each cycle in {@link #periodic()}, and provides
 * generic {@link #extendCommand()} / {@link #retractCommand()} implementations.
 *
 * <p>Automatically binds state to the web UI (if active) via {@link WebControl#getInstance()}
 * so subclasses never need to inject or reference {@link WebControl} directly.
 *
 * <p>Subclasses supply game-specific behaviour by overriding {@link #getScoredCount()}
 * and adding firing / scoring logic. Access {@link #intakeSimulation} and
 * {@link #intakeExtended} directly from the subclass as needed.
 */
public abstract class SimRobotState extends SubsystemBase {

  protected final IntakeSimulation intakeSimulation;
  /** Set by {@link #extendCommand()} / {@link #retractCommand()}; read by the HTTP thread. */
  protected volatile boolean intakeExtended = false;

  protected SimRobotState(IntakeSimulation intakeSimulation) {
    this.intakeSimulation = intakeSimulation;
    intakeSimulation.register();
    WebControl.getInstance().ifPresent(wc ->
        wc.bindDemoState(this::getHeldPieces, this::isExtended, this::getScoredCount));
  }

  @Override
  public void periodic() {
    intakeSimulation.removeObtainedGamePieces(SimulatedArena.getInstance());
  }

  // ---- state — implementations are thread-safe for HTTP thread reads ----

  protected int getHeldPieces()  { return intakeSimulation.getGamePiecesAmount(); }
  protected boolean isExtended() { return intakeExtended; }
  /** Override to report game-specific scored count. Default returns 0. */
  protected int getScoredCount() { return 0; }

  // ---- generic intake commands ----

  public Command extendCommand() {
    return Commands.runOnce(() -> {
      intakeSimulation.startIntake();
      intakeExtended = true;
    }, this).withName("ExtendIntake");
  }

  public Command retractCommand() {
    return Commands.runOnce(() -> {
      intakeSimulation.stopIntake();
      intakeExtended = false;
    }, this).withName("RetractIntake");
  }
}
