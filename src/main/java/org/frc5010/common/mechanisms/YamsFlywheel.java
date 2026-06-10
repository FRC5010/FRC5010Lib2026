package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.LQRConfig;
import yams.math.LQRController;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * LQR-controlled flywheel (shooter wheel, intake roller) built on the YAMS
 * {@link FlyWheel} velocity mechanism.
 *
 * <p><b>Controller:</b> FLYWHEEL-type LQR — a 1-state velocity loop with Kalman filter.
 * The {@code LinearSystemLoop} includes plant-inversion feedforward, so no kV
 * {@code SimpleMotorFeedforward} is configured (it would double-apply).
 * Motion profiles are ignored by YAMS for flywheel velocity LQR.
 *
 * <p>Robot-specific values live in {@link Settings}
 * (see {@code frc.robot.mechanisms.ExampleShooter}). The velocity error tolerance and
 * control effort are live-tunable under {@code /Tuning/<name>/} ({@link LqrTunables};
 * the position weight is unused for flywheels).
 */
public class YamsFlywheel extends SubsystemBase {

  /** Robot-specific flywheel parameters. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "Flywheel";
    /** Motor controller vendor. */
    public MechanismMotor.Vendor vendor = MechanismMotor.Vendor.TALON_FX;
    /** CAN ID of the motor controller. */
    public int canId;
    /** Motor physics model. */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → wheel (1.0 = direct drive). */
    public double[] gearReductionStages = {1.0};
    /** Flywheel diameter (used for MOI estimate and exit-velocity telemetry). */
    public Distance diameter = Inches.of(4);
    /** Flywheel mass. */
    public Mass mass = Kilograms.of(1.5);

    // --- LQR weights (live-tunable; these are the initial values) ---
    /** Velocity error tolerance. Smaller = more aggressive. */
    public AngularVelocity qelmsVelocity = RadiansPerSecond.of(8);
    /** Control effort tolerance. Smaller = gentler. 12 V = full battery. */
    public Voltage relms = Volts.of(12);

    // --- Kalman filter trust (rarely changed) ---
    /** Model velocity standard deviation. */
    public AngularVelocity modelVelocityTrust = RadiansPerSecond.of(3.0);
    /** Encoder velocity standard deviation. */
    public AngularVelocity encoderVelocityTrust = RadiansPerSecond.of(0.01);

    /** Stator current limit. */
    public Current statorCurrentLimit = Amps.of(60);
  }

  private final Settings settings;
  private final MechanismGearing gearing;
  private final LQRController lqr;
  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController motor;
  private final FlyWheel flywheel;
  private final LqrTunables tunables;

  /**
   * Builds the flywheel subsystem, motor wrapper, LQR controller, and simulation.
   *
   * @param settings robot-specific flywheel parameters
   */
  public YamsFlywheel(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    gearing = new MechanismGearing(GearBox.fromReductionStages(settings.gearReductionStages));
    lqr = new LQRController(buildLqrConfig(
        settings.qelmsVelocity.in(RotationsPerSecond),
        settings.relms.in(Volts)));
    motorConfig = new SmartMotorControllerConfig(this)
        .withGearing(gearing)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(settings.statorCurrentLimit)
        .withTelemetry(settings.name + "Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Must stay LAST: the PID-style withClosedLoopController overloads clear the LQR.
        .withClosedLoopController(lqr);
    motor = MechanismMotor.create(settings.vendor, settings.canId, settings.motorModel, motorConfig);
    flywheel = new FlyWheel(new FlyWheelConfig(motor)
        .withDiameter(settings.diameter)
        .withMass(settings.mass)
        .withTelemetry(settings.name, TelemetryVerbosity.HIGH));
    tunables = new LqrTunables(settings.name,
        0, // position weight unused for flywheel LQR
        settings.qelmsVelocity.in(RotationsPerSecond),
        settings.relms.in(Volts));
  }

  private LQRConfig buildLqrConfig(double qelmsVelRps, double relmsVolts) {
    // Same MOI estimate the YAMS FlyWheel sim uses (SingleJointedArmSim.estimateMOI),
    // so the LQR plant matches the simulated plant exactly.
    double moi = settings.mass.in(Kilograms) * Math.pow(settings.diameter.in(Meters), 2) / 3.0;
    return MechanismLqrConfig.flywheel(
        settings.motorModel,
        gearing,
        KilogramSquareMeters.of(moi),
        RotationsPerSecond.of(qelmsVelRps),
        settings.modelVelocityTrust,
        settings.encoderVelocityTrust,
        Volts.of(relmsVolts));
  }

  @Override
  public void periodic() {
    if (tunables.hasChanged()) {
      lqr.updateConfig(buildLqrConfig(tunables.qelmsVelocity(), tunables.relms()));
      motor.startClosedLoopController();
    }
    flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    flywheel.simIterate();
  }

  /** Command: spin the wheel to the given velocity (LQR). Never finishes. */
  public Command goToSpeed(AngularVelocity speed) {
    return flywheel.setSpeed(speed);
  }

  /** Command: open-loop duty cycle. */
  public Command setDutyCycle(double dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  /** Command: SysId routine for characterizing kS/kV on a real robot. */
  public Command sysId() {
    return flywheel.sysId(Volts.of(7), Volts.of(1).per(Second), Seconds.of(10));
  }

  /** Current wheel speed. */
  public AngularVelocity getSpeed() {
    return flywheel.getSpeed();
  }

  /** Trigger: true while the wheel is within {@code tolerance} of {@code speed} (ready to shoot). */
  public Trigger isAtSpeed(AngularVelocity speed, AngularVelocity tolerance) {
    return flywheel.isNear(speed, tolerance);
  }

  /** Convenience trigger with a 100 RPM tolerance. */
  public Trigger isAtSpeed(AngularVelocity speed) {
    return isAtSpeed(speed, RPM.of(100));
  }

  /** Underlying YAMS mechanism, for advanced use. */
  public FlyWheel getMechanism() {
    return flywheel;
  }

  /** Underlying YAMS motor wrapper, for advanced use. */
  public SmartMotorController getMotor() {
    return motor;
  }

  /** Stops the closed-loop Notifier and frees the CAN device. For unit tests. */
  public void close() {
    motor.close();
    MechanismMotor.closeDevice(motor);
  }
}
