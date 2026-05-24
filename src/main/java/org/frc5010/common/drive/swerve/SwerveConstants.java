package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Immutable configuration record for a swerve drivetrain.
 *
 * <p>Students fill this out once per robot in their Constants.java and pass it
 * to SwerveFactory. All physical dimensions are in meters, all angles in radians,
 * all speeds in meters per second unless noted.
 *
 * <p>Example usage:
 * <pre>{@code
 * public static final SwerveConstants DRIVE = new SwerveConstants.Builder()
 *     .trackWidthMeters(Units.inchesToMeters(22.75))
 *     .wheelBaseMeters(Units.inchesToMeters(22.75))
 *     .wheelRadiusMeters(Units.inchesToMeters(2.0))
 *     .maxLinearSpeedMps(4.5)
 *     .maxAngularSpeedRadps(2 * Math.PI)
 *     .moduleType(ModuleType.TALON_FX)
 *     .gyroType(GyroType.PIGEON2)
 *     .gyroCanId(0)
 *     .frontLeftIds(1, 2, 3)
 *     .frontRightIds(4, 5, 6)
 *     .backLeftIds(7, 8, 9)
 *     .backRightIds(10, 11, 12)
 *     .build();
 * }</pre>
 */
public final class SwerveConstants {

  /** Supported swerve module hardware configurations. */
  public enum ModuleType {
    TALON_FX,        // Falcon 500 or Kraken X60 drive + steer
    SPARK_MAX,       // NEO drive + steer via SparkMax
    SPARK_TALON,     // NEO drive (SparkMax) + Falcon steer (TalonFX)
    SIM              // Simulation only — selected automatically in SIM mode
  }

  /** Supported gyro hardware. */
  public enum GyroType {
    PIGEON2,
    NAVX,
    SIM
  }

  // --- Physical geometry ---
  public final double trackWidthMeters;
  public final double wheelBaseMeters;
  public final double wheelRadiusMeters;

  // --- Performance limits ---
  public final double maxLinearSpeedMps;
  public final double maxAngularSpeedRadps;

  // --- Hardware selection ---
  public final ModuleType moduleType;
  public final GyroType gyroType;
  public final int gyroCanId;

  // --- CAN IDs: [driveId, steerId, encoderId] per module ---
  public final int[] frontLeftIds;
  public final int[] frontRightIds;
  public final int[] backLeftIds;
  public final int[] backRightIds;

  // --- CAN bus and odometry ---
  /** Phoenix CANivore bus name. Use {@code ""} for the default RIO bus. */
  public final String canBusName;
  /** High-frequency odometry update rate in Hz (e.g., 100 for typical CAN, 250 for CANivore). */
  public final double odometryFrequencyHz;

  // --- Physics simulation parameters ---
  /**
   * Total robot mass in kilograms, including bumpers and battery.
   * Used by the IronMaple physics engine in SIM mode.
   * Valid range: 10–80 kg (FRC robot weight limits; IronMaple enforces this).
   */
  public final double robotMassKg;
  /**
   * Bumper length in meters — full outside-to-outside dimension from front to back.
   * Used by the IronMaple physics engine for collision geometry in SIM mode.
   * Valid range: 0.5–1.5 m.
   */
  public final double bumperLengthMeters;
  /**
   * Bumper width in meters — full outside-to-outside dimension from left to right.
   * Used by the IronMaple physics engine for collision geometry in SIM mode.
   * Valid range: 0.5–1.5 m.
   */
  public final double bumperWidthMeters;

  // --- Derived geometry (computed once at construction) ---
  public final Translation2d[] moduleTranslations;

  private SwerveConstants(Builder b) {
    this.trackWidthMeters   = b.trackWidthMeters;
    this.wheelBaseMeters    = b.wheelBaseMeters;
    this.wheelRadiusMeters  = b.wheelRadiusMeters;
    this.maxLinearSpeedMps  = b.maxLinearSpeedMps;
    this.maxAngularSpeedRadps = b.maxAngularSpeedRadps;
    this.moduleType         = b.moduleType;
    this.gyroType           = b.gyroType;
    this.gyroCanId          = b.gyroCanId;
    this.frontLeftIds       = b.frontLeftIds;
    this.frontRightIds      = b.frontRightIds;
    this.backLeftIds        = b.backLeftIds;
    this.backRightIds       = b.backRightIds;
    this.canBusName         = b.canBusName;
    this.odometryFrequencyHz = b.odometryFrequencyHz;
    this.robotMassKg        = b.robotMassKg;
    this.bumperLengthMeters = b.bumperLengthMeters;
    this.bumperWidthMeters  = b.bumperWidthMeters;

    // Compute module positions relative to robot center.
    // Order matches WPILib convention: FL, FR, BL, BR.
    double x = wheelBaseMeters / 2.0;
    double y = trackWidthMeters / 2.0;
    this.moduleTranslations = new Translation2d[] {
      new Translation2d( x,  y),  // Front Left
      new Translation2d( x, -y),  // Front Right
      new Translation2d(-x,  y),  // Back Left
      new Translation2d(-x, -y),  // Back Right
    };
  }

  /**
   * Validates that this configuration is internally consistent.
   * Called by the Builder before constructing the object.
   *
   * @throws IllegalArgumentException if any value is out of range
   */
  private void validate() {
    if (trackWidthMeters <= 0)
      throw new IllegalArgumentException("trackWidthMeters must be > 0, got: " + trackWidthMeters);
    if (wheelBaseMeters <= 0)
      throw new IllegalArgumentException("wheelBaseMeters must be > 0, got: " + wheelBaseMeters);
    if (wheelRadiusMeters <= 0)
      throw new IllegalArgumentException("wheelRadiusMeters must be > 0, got: " + wheelRadiusMeters);
    if (maxLinearSpeedMps <= 0)
      throw new IllegalArgumentException("maxLinearSpeedMps must be > 0, got: " + maxLinearSpeedMps);
    if (maxAngularSpeedRadps <= 0)
      throw new IllegalArgumentException("maxAngularSpeedRadps must be > 0, got: " + maxAngularSpeedRadps);
    if (moduleType == null)
      throw new IllegalArgumentException("moduleType must not be null");
    if (gyroType == null)
      throw new IllegalArgumentException("gyroType must not be null");
    if (frontLeftIds == null || frontLeftIds.length < 2)
      throw new IllegalArgumentException("frontLeftIds must have at least [driveId, steerId]");
    if (frontRightIds == null || frontRightIds.length < 2)
      throw new IllegalArgumentException("frontRightIds must have at least [driveId, steerId]");
    if (backLeftIds == null || backLeftIds.length < 2)
      throw new IllegalArgumentException("backLeftIds must have at least [driveId, steerId]");
    if (backRightIds == null || backRightIds.length < 2)
      throw new IllegalArgumentException("backRightIds must have at least [driveId, steerId]");
    if (robotMassKg < 10 || robotMassKg > 80)
      throw new IllegalArgumentException(
          "robotMassKg must be 10–80 kg (FRC weight limits), got: " + robotMassKg);
    if (bumperLengthMeters < 0.5 || bumperLengthMeters > 1.5)
      throw new IllegalArgumentException(
          "bumperLengthMeters must be 0.5–1.5 m, got: " + bumperLengthMeters);
    if (bumperWidthMeters < 0.5 || bumperWidthMeters > 1.5)
      throw new IllegalArgumentException(
          "bumperWidthMeters must be 0.5–1.5 m, got: " + bumperWidthMeters);
  }

  /** Fluent builder for SwerveConstants. */
  public static class Builder {
    private double trackWidthMeters   = Units.inchesToMeters(24);
    private double wheelBaseMeters    = Units.inchesToMeters(24);
    private double wheelRadiusMeters  = Units.inchesToMeters(2);
    private double maxLinearSpeedMps  = 4.5;
    private double maxAngularSpeedRadps = 2 * Math.PI;
    private ModuleType moduleType     = ModuleType.SIM;
    private GyroType gyroType         = GyroType.SIM;
    private int gyroCanId             = 0;
    private int[] frontLeftIds        = {1, 2, 3};
    private int[] frontRightIds       = {4, 5, 6};
    private int[] backLeftIds         = {7, 8, 9};
    private int[] backRightIds        = {10, 11, 12};
    private String canBusName         = "";
    private double odometryFrequencyHz = 100.0;
    // Physics simulation — defaults match IronMaple's DriveTrainSimulationConfig.Default()
    private double robotMassKg        = 45.0;
    private double bumperLengthMeters = 0.76;
    private double bumperWidthMeters  = 0.76;

    public Builder trackWidthMeters(double v)     { trackWidthMeters = v; return this; }
    public Builder wheelBaseMeters(double v)      { wheelBaseMeters = v; return this; }
    public Builder wheelRadiusMeters(double v)    { wheelRadiusMeters = v; return this; }
    public Builder maxLinearSpeedMps(double v)    { maxLinearSpeedMps = v; return this; }
    public Builder maxAngularSpeedRadps(double v) { maxAngularSpeedRadps = v; return this; }
    public Builder moduleType(ModuleType v)       { moduleType = v; return this; }
    public Builder gyroType(GyroType v)           { gyroType = v; return this; }
    public Builder gyroCanId(int v)               { gyroCanId = v; return this; }

    /** Set drive, steer, and encoder CAN IDs for front-left module. */
    public Builder frontLeftIds(int drive, int steer, int encoder) {
      frontLeftIds = new int[]{drive, steer, encoder}; return this;
    }
    /** Set drive and steer CAN IDs (no separate encoder) for front-left module. */
    public Builder frontLeftIds(int drive, int steer) {
      frontLeftIds = new int[]{drive, steer}; return this;
    }
    public Builder frontRightIds(int drive, int steer, int encoder) {
      frontRightIds = new int[]{drive, steer, encoder}; return this;
    }
    public Builder frontRightIds(int drive, int steer) {
      frontRightIds = new int[]{drive, steer}; return this;
    }
    public Builder backLeftIds(int drive, int steer, int encoder) {
      backLeftIds = new int[]{drive, steer, encoder}; return this;
    }
    public Builder backLeftIds(int drive, int steer) {
      backLeftIds = new int[]{drive, steer}; return this;
    }
    public Builder backRightIds(int drive, int steer, int encoder) {
      backRightIds = new int[]{drive, steer, encoder}; return this;
    }
    public Builder backRightIds(int drive, int steer) {
      backRightIds = new int[]{drive, steer}; return this;
    }
    /** Phoenix CANivore bus name. Use {@code ""} for the default RIO bus. */
    public Builder canBusName(String v) { canBusName = v; return this; }
    /** High-frequency odometry rate in Hz (100 for standard CAN, 250 for CANivore). */
    public Builder odometryFrequencyHz(double v) { odometryFrequencyHz = v; return this; }

    /**
     * Total robot mass in kilograms, including bumpers and battery (10–80 kg).
     * Passed to the IronMaple physics engine so the sim reflects actual robot weight.
     * Default: 45 kg.
     */
    public Builder robotMassKg(double v) { robotMassKg = v; return this; }
    /**
     * Full bumper length in meters, front to back outside-to-outside (0.5–1.5 m).
     * Used for physics collision geometry. Default: 0.76 m (~30 in).
     */
    public Builder bumperLengthMeters(double v) { bumperLengthMeters = v; return this; }
    /**
     * Full bumper width in meters, left to right outside-to-outside (0.5–1.5 m).
     * Used for physics collision geometry. Default: 0.76 m (~30 in).
     */
    public Builder bumperWidthMeters(double v) { bumperWidthMeters = v; return this; }

    public SwerveConstants build() {
      SwerveConstants c = new SwerveConstants(this);
      c.validate();
      return c;
    }
  }
}