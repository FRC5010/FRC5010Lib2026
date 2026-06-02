package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.profiles.RobotProfile;
import org.frc5010.common.profiles.SimRobotProfile;
import org.frc5010.common.profiles.SwerveRobotContainer;

/**
 * Top-level robot container — wired by {@link Robot} on startup.
 *
 * <p>Selects the robot profile based on context:
 * <ul>
 *   <li>{@code RobotBase.isReal()} → {@link RealRobotProfile} (hardware IO)</li>
 *   <li>Simulation → {@link SimRobotProfile} by default (test robot, no real CAN IDs)</li>
 * </ul>
 *
 * <p>All wiring — keyboard drive, alliance reset, vision, visual test — is handled by
 * {@link SwerveRobotContainer} (drive) and {@link RealRobotProfile#createVision} (cameras).
 * This subclass adds the {@link DemoIntake} demo, wiring its three inputs to both the
 * web UI buttons (when {@code -PwebUI} is set) and the keyboard/Xbox controller so the
 * demo is interactive from either UI.
 */
public class RobotContainer extends SwerveRobotContainer {

  private DemoIntake demoIntake;

  public RobotContainer() {
    super(selectProfile());
  }

  private static RobotProfile selectProfile() {
    if (RobotBase.isReal()) return new RealRobotProfile();
    if (Boolean.getBoolean("testSim")) return new SimRobotProfile();
    return new RealRobotProfile();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();
    if (!RobotBase.isSimulation()) return;

    // Controller buttons match WPILib XboxController constants: A=1, LB=5, RB=6.
    // Web indices (from WebDriveController.getButton): A=0, LB=4, RB=5.
    // OR-ing both sources lets the same demo work from Glass keyboard, an Xbox
    // controller, or the web UI — whichever is in use.
    demoIntake = new DemoIntake(
        () -> controller.button(5).getAsBoolean() || webButton(4).getAsBoolean(),
        () -> controller.button(6).getAsBoolean() || webButton(5).getAsBoolean(),
        () -> controller.button(1).getAsBoolean() || webButton(0).getAsBoolean());

    if (webController != null) {
      webController.bindDemoState(
          demoIntake::getHeldFuel,
          demoIntake::isIntakeExtended,
          demoIntake::getScoredCount);
    }

    // Run DemoIntake.periodic() every enabled cycle. Requires no subsystem so it
    // doesn't conflict with the drive's default command.
    CommandScheduler.getInstance().schedule(
        Commands.run(() -> demoIntake.periodic(drive.getPose()))
            .withName("DemoIntakePeriodic"));
  }
}
