package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.frc5010.common.profiles.SwerveRobotContainer;

/**
 * Team-specific robot container — extends {@link SwerveRobotContainer} with
 * competition hardware constants ({@link RealRobotProfile}) and the {@link DemoIntake} demo.
 *
 * <p>Profile selection is handled by {@link SwerveRobotContainer#selectProfile(String)}: pass
 * the fully-qualified class name of the desired profile. To use a custom profile that extends
 * {@link RealRobotProfile}, just change the class name string here.
 *
 * <p>Note: {@link SwerveRobotContainer}'s constructor calls {@link #configureBindings()} before
 * this class's constructor body runs. Fields initialised after {@code super(...)} would be
 * {@code null} when {@code configureBindings()} fires — initialise such fields inside
 * {@code configureBindings()} itself (as {@code demoIntake} is here).
 */
public class RealRobot extends SwerveRobotContainer {

  private DemoIntake demoIntake;

  public RealRobot() {
    super(SwerveRobotContainer.selectProfile("frc.robot.RealRobotProfile"));
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();
    if (!RobotBase.isSimulation()) return;

    // Controller buttons: A=1, LB=5, RB=6 (WPILib XboxController constants).
    // Web indices (WebDriveController.getButton): A=0, LB=4, RB=5.
    // OR-ing both sources lets the demo work from Glass keyboard or the web UI.
    demoIntake = new DemoIntake(
        drive::getPose,
        () -> controller.button(5).getAsBoolean() || webButton(4).getAsBoolean(),
        () -> controller.button(6).getAsBoolean() || webButton(5).getAsBoolean(),
        () -> controller.button(1).getAsBoolean() || webButton(0).getAsBoolean());

    if (webController != null) {
      webController.bindDemoState(
          demoIntake::getHeldFuel,
          demoIntake::isIntakeExtended,
          demoIntake::getScoredCount);
    }
    // No manual scheduling — DemoIntake sets its own default command in its constructor.
  }
}
