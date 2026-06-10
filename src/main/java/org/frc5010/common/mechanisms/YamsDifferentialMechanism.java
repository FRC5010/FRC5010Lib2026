package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.tuning.TunableGains;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.DifferentialMechanismConfig;
import yams.mechanisms.positional.DifferentialMechanism;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Differential (tilt + twist) wrist built on the YAMS {@link DifferentialMechanism}.
 *
 * <p>Two motors drive one gear assembly: spinning together produces <em>tilt</em>,
 * spinning opposite produces <em>twist</em>.
 *
 * <p><b>Controller:</b> profiled PID per motor, <em>not</em> LQR — the YAMS LQR supports
 * only single-DOF FLYWHEEL / ARM / ELEVATOR plants, and the differential couples two
 * motors into two output axes. Gains are shared by both motors (the mechanism is
 * symmetric) and live-tunable under {@code /Tuning/<name>/} via {@link TunableGains}.
 *
 * <p>Robot-specific values live in {@link Settings}
 * (see {@code frc.robot.mechanisms.ExampleDifferentialWrist}).
 */
public class YamsDifferentialMechanism extends SubsystemBase {

  /** Robot-specific differential mechanism parameters. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "DiffWrist";
    /** Motor controller vendor (both motors). */
    public MechanismMotor.Vendor vendor = MechanismMotor.Vendor.TALON_FX;
    /** CAN ID of the left motor controller. */
    public int leftCanId;
    /** CAN ID of the right motor controller. */
    public int rightCanId;
    /** Motor physics model (both motors). */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → mechanism (both motors). */
    public double[] gearReductionStages = {3, 4, 5};
    /** Effective lever length for the MOI estimate. */
    public Distance length = Meters.of(0.3);
    /** Mass carried by the wrist for the MOI estimate. */
    public Mass mass = Kilograms.of(1.8);
    /** Tilt angle at robot power-on. */
    public Angle startingTilt = Degrees.of(90);
    /** Twist angle at robot power-on. */
    public Angle startingTwist = Degrees.of(0);
    /** Proportional gain, both motors (rotations → volts). */
    public double kP = 16;
    /** Integral gain. */
    public double kI = 0;
    /** Derivative gain. */
    public double kD = 0;
    /** Motion profile cruise velocity. */
    public AngularVelocity maxVelocity = DegreesPerSecond.of(180);
    /** Motion profile acceleration. */
    public AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(360);
    /** Stator current limit (both motors). */
    public Current statorCurrentLimit = Amps.of(40);
  }

  private final Settings settings;
  private final SmartMotorControllerConfig leftMotorConfig;
  private final SmartMotorControllerConfig rightMotorConfig;
  private final SmartMotorController leftMotor;
  private final SmartMotorController rightMotor;
  private final DifferentialMechanism diffy;
  private final TunableGains gains;

  /**
   * Builds the differential mechanism subsystem, both motor wrappers, and simulation.
   *
   * @param settings robot-specific parameters
   */
  public YamsDifferentialMechanism(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    leftMotorConfig = motorConfig("Left");
    rightMotorConfig = motorConfig("Right");
    leftMotor = MechanismMotor.create(
        settings.vendor, settings.leftCanId, settings.motorModel, leftMotorConfig);
    rightMotor = MechanismMotor.create(
        settings.vendor, settings.rightCanId, settings.motorModel, rightMotorConfig);
    diffy = new DifferentialMechanism(new DifferentialMechanismConfig(leftMotor, rightMotor)
        .withStartingPosition(settings.startingTilt, settings.startingTwist)
        .withMOI(settings.length, settings.mass)
        .withTelemetry(settings.name, TelemetryVerbosity.HIGH));
    gains = new TunableGains(settings.name, "motors",
        settings.kP, settings.kI, settings.kD, 0);
  }

  private SmartMotorControllerConfig motorConfig(String label) {
    return new SmartMotorControllerConfig(this)
        .withClosedLoopController(settings.kP, settings.kI, settings.kD)
        .withTrapezoidalProfile(settings.maxVelocity, settings.maxAcceleration)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(settings.gearReductionStages)))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(settings.statorCurrentLimit)
        .withTelemetry(settings.name + label + "Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP);
  }

  @Override
  public void periodic() {
    if (gains.hasChanged()) {
      leftMotorConfig.withClosedLoopController(gains.kP(), gains.kI(), gains.kD());
      rightMotorConfig.withClosedLoopController(gains.kP(), gains.kI(), gains.kD());
      leftMotor.applyConfig(leftMotorConfig);
      rightMotor.applyConfig(rightMotorConfig);
    }
    diffy.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    diffy.simIterate();
  }

  /** Command: drive the wrist to the given tilt and twist angles. Never finishes. */
  public Command goToAngles(Angle tilt, Angle twist) {
    return diffy.setPosition(tilt, twist);
  }

  /** Command: open-loop duty cycle (tilt = common mode, twist = differential mode). */
  public Command setDutyCycle(double tilt, double twist) {
    return diffy.set(tilt, twist);
  }

  /** Command: SysId routine. */
  public Command sysId() {
    return diffy.sysId(Volts.of(3), Volts.of(1).per(Second), Seconds.of(10));
  }

  /** Current tilt angle. */
  public Angle getTilt() {
    return diffy.getTiltPosition();
  }

  /** Current twist angle. */
  public Angle getTwist() {
    return diffy.getTwistPosition();
  }

  /** Underlying YAMS mechanism, for advanced use. */
  public DifferentialMechanism getMechanism() {
    return diffy;
  }

  /** Stops the closed-loop Notifiers and frees the CAN devices. For unit tests. */
  public void close() {
    leftMotor.close();
    rightMotor.close();
    MechanismMotor.closeDevice(leftMotor);
    MechanismMotor.closeDevice(rightMotor);
  }
}
