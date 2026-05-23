package org.frc5010.common.robot;

/**
 * Robot operating mode, set once at startup and used throughout the framework
 * to select the correct IO implementations.
 *
 * <p>REAL    — running on physical hardware (roboRIO + actual motors/sensors)
 * <p>SIM     — running in WPILib simulation (desktop, no hardware)
 * <p>REPLAY  — replaying a previously recorded AdvantageKit log file
 */
public enum Mode {
  REAL,
  SIM,
  REPLAY
}