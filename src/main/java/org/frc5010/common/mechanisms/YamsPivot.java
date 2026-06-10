package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.LQRConfig;
import yams.math.LQRController;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * LQR-controlled pivot (turret, hood, wrist) built on the YAMS {@link Pivot} mechanism.
 *
 * <p>A pivot is a gravity-free rotating mechanism — same plant as an arm but without the
 * gravity feedforward. The ARM-type LQR (motor model + gearing + MOI) is the correct
 * state-space model for it; YAMS has no separate PIVOT LQR type.
 *
 * <p>Robot-specific values live in {@link Settings}
 * (see {@code frc.robot.mechanisms.ExampleTurret}). LQR weights are live-tunable under
 * {@code /Tuning/<name>/} ({@link LqrTunables}; rotations / rotations-per-second).
 */
public class YamsPivot extends SubsystemBase {

  /** Robot-specific pivot parameters. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "Pivot";
    /** Motor controller vendor. */
    public MechanismMotor.Vendor vendor = MechanismMotor.Vendor.TALON_FX;
    /** CAN ID of the motor controller. */
    public int canId;
    /** Motor physics model. */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → mechanism (e.g. {10, 4} = 40:1). */
    public double[] gearReductionStages = {10, 4};
    /** Moment of inertia of the rotating assembly about the pivot axis. */
    public MomentOfInertia moi = KilogramSquareMeters.of(0.5);
    /** Lower hard limit. */
    public Angle minAngle = Degrees.of(-180);
    /** Upper hard limit. */
    public Angle maxAngle = Degrees.of(180);
    /** Pivot angle at robot power-on. */
    public Angle startingAngle = Degrees.of(0);
    /** Motion profile cruise velocity. */
    public AngularVelocity maxVelocity = DegreesPerSecond.of(360);
    /** Motion profile acceleration. */
    public AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(720);
    /** Stator current limit. */
    public Current statorCurrentLimit = Amps.of(40);

    // --- LQR weights (live-tunable; these are the initial values) ---
    /** Position error tolerance. Smaller = more aggressive. */
    public Angle qelmsPosition = Degrees.of(1.0);
    /** Velocity error tolerance. Smaller = more aggressive. */
    public AngularVelocity qelmsVelocity = DegreesPerSecond.of(20);
    /** Control effort tolerance. Smaller = gentler. 12 V = full battery. */
    public Voltage relms = Volts.of(12);

    // --- Kalman filter trust (rarely changed) ---
    /** Model position standard deviation. */
    public Angle modelPositionTrust = Radians.of(0.015);
    /** Model velocity standard deviation. */
    public AngularVelocity modelVelocityTrust = RadiansPerSecond.of(0.17);
    /** Encoder position standard deviation. */
    public Angle encoderPositionTrust = Radians.of(0.001);
  }

  private final Settings settings;
  private final MechanismGearing gearing;
  private final LQRController lqr;
  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController motor;
  private final Pivot pivot;
  private final LqrTunables tunables;

  /**
   * Builds the pivot subsystem, motor wrapper, LQR controller, and simulation.
   *
   * @param settings robot-specific pivot parameters
   */
  public YamsPivot(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    gearing = new MechanismGearing(GearBox.fromReductionStages(settings.gearReductionStages));
    lqr = new LQRController(buildLqrConfig(
        settings.qelmsPosition.in(Rotations),
        settings.qelmsVelocity.in(RotationsPerSecond),
        settings.relms.in(Volts)));
    motorConfig = new SmartMotorControllerConfig(this)
        .withGearing(gearing)
        .withSoftLimit(settings.minAngle, settings.maxAngle)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(settings.statorCurrentLimit)
        .withTelemetry(settings.name + "Motor", TelemetryVerbosity.HIGH)
        .withTrapezoidalProfile(settings.maxVelocity, settings.maxAcceleration)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Must stay LAST: the PID-style withClosedLoopController overloads clear the LQR.
        .withClosedLoopController(lqr);
    motor = MechanismMotor.create(settings.vendor, settings.canId, settings.motorModel, motorConfig);
    pivot = new Pivot(new PivotConfig(motor)
        .withMOI(settings.moi)
        .withHardLimit(settings.minAngle, settings.maxAngle)
        .withStartingPosition(settings.startingAngle)
        .withTelemetry(settings.name, TelemetryVerbosity.HIGH));
    tunables = new LqrTunables(settings.name,
        settings.qelmsPosition.in(Rotations),
        settings.qelmsVelocity.in(RotationsPerSecond),
        settings.relms.in(Volts));
  }

  private LQRConfig buildLqrConfig(double qelmsPosRot, double qelmsVelRps, double relmsVolts) {
    return MechanismLqrConfig.arm(
        settings.motorModel,
        gearing,
        settings.moi,
        Rotations.of(qelmsPosRot),
        RotationsPerSecond.of(qelmsVelRps),
        settings.modelPositionTrust,
        settings.modelVelocityTrust,
        settings.encoderPositionTrust,
        Volts.of(relmsVolts));
  }

  @Override
  public void periodic() {
    if (tunables.hasChanged()) {
      lqr.updateConfig(buildLqrConfig(
          tunables.qelmsPosition(), tunables.qelmsVelocity(), tunables.relms()));
      motor.startClosedLoopController();
    }
    pivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    pivot.simIterate();
  }

  /** Command: rotate the pivot to the given angle (profiled LQR). Never finishes. */
  public Command goToAngle(Angle angle) {
    return pivot.setAngle(angle);
  }

  /** Command: open-loop duty cycle (e.g. for manual jog). */
  public Command setDutyCycle(double dutyCycle) {
    return pivot.set(dutyCycle);
  }

  /** Command: SysId routine for characterizing kS/kV on a real robot. */
  public Command sysId() {
    return pivot.sysId(Volts.of(3), Volts.of(1).per(Second), Seconds.of(10));
  }

  /** Current pivot angle. */
  public Angle getAngle() {
    return pivot.getAngle();
  }

  /** Trigger: true while the pivot is within {@code tolerance} of {@code angle}. */
  public Trigger isAtAngle(Angle angle, Angle tolerance) {
    return pivot.isNear(angle, tolerance);
  }

  /** Underlying YAMS mechanism, for advanced use. */
  public Pivot getMechanism() {
    return pivot;
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
