package org.frc5010.common.input;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An {@link XboxConfigurableController} extended with web UI button OR-ing.
 *
 * <p>All named button accessors ({@link #a()}, {@link #leftBumper()}, etc.) automatically
 * OR the physical button with its web UI equivalent once {@link #setWebInputs} has been
 * called by {@link org.frc5010.common.sim.WebControl}.
 *
 * <p>Only instantiated when {@code -PwebUI} is set. For real hardware or plain simulation
 * without a web UI, {@link XboxConfigurableController} is used directly.
 */
public class WebXboxController extends XboxConfigurableController {

  // Maps WPILib 1-based button index → 0-based web UI button index; -1 = no web equivalent.
  private static final int[] WPILIB_TO_WEB = {
      -1,  // 0: unused
       0,  // 1: A
       1,  // 2: B
       2,  // 3: X
       3,  // 4: Y
       4,  // 5: LB
       5,  // 6: RB
      -1,  // 7: Back
      -1,  // 8: Start
      -1,  // 9: LeftStick
      -1,  // 10: RightStick
  };

  private BooleanSupplier[] webInputs = null;

  /**
   * Creates a web-aware Xbox controller on the specified Driver Station port.
   *
   * @param port WPILib joystick port (0–5)
   */
  public WebXboxController(int port) {
    super(port);
  }

  /**
   * Injects web UI button suppliers. Called by {@link org.frc5010.common.sim.WebControl}
   * after the web server starts, before any button bindings are created.
   */
  public void setWebInputs(BooleanSupplier[] inputs) {
    this.webInputs = inputs;
  }

  /**
   * Returns a {@link Trigger} for button {@code index}, automatically OR-ing the physical
   * button with its web UI equivalent when web inputs have been injected. All named
   * accessors ({@link #a()}, {@link #leftBumper()}, etc.) route through this method.
   */
  @Override
  public Trigger button(int index) {
    Trigger physical = super.button(index);
    if (webInputs != null && index >= 1 && index < WPILIB_TO_WEB.length) {
      int webIdx = WPILIB_TO_WEB[index];
      if (webIdx >= 0 && webIdx < webInputs.length) {
        return physical.or(new Trigger(webInputs[webIdx]));
      }
    }
    return physical;
  }
}
