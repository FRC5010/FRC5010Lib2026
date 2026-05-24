package org.frc5010.common.unit;

import static org.junit.jupiter.api.Assertions.*;

import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

/**
 * Layer 1 unit tests for SwerveConstants.
 * No HAL, no WPILib sim — pure Java logic only.
 */
class SwerveConstantsTest {

  /** A valid baseline config used across multiple tests. */
  private SwerveConstants.Builder validBuilder() {
    return new SwerveConstants.Builder()
        .trackWidthMeters(Units.inchesToMeters(22.75))
        .wheelBaseMeters(Units.inchesToMeters(22.75))
        .wheelRadiusMeters(Units.inchesToMeters(2.0))
        .maxLinearSpeedMps(4.5)
        .maxAngularSpeedRadps(2 * Math.PI)
        .moduleType(ModuleType.SIM)
        .gyroType(GyroType.SIM)
        .frontLeftIds(1, 2, 3)
        .frontRightIds(4, 5, 6)
        .backLeftIds(7, 8, 9)
        .backRightIds(10, 11, 12);
  }

  @Test
  void validConfigBuildsSuccessfully() {
    assertDoesNotThrow(() -> validBuilder().build());
  }

  @Test
  void moduleTranslationsHaveCorrectCount() {
    SwerveConstants c = validBuilder().build();
    assertEquals(4, c.moduleTranslations.length);
  }

  @Test
  void moduleTranslationsReflectGeometry() {
    double trackWidth = Units.inchesToMeters(22.75);
    double wheelBase  = Units.inchesToMeters(22.75);
    SwerveConstants c = validBuilder()
        .trackWidthMeters(trackWidth)
        .wheelBaseMeters(wheelBase)
        .build();

    double expectedX = wheelBase / 2.0;
    double expectedY = trackWidth / 2.0;

    // Front Left: (+x, +y)
    assertEquals( expectedX, c.moduleTranslations[0].getX(), 1e-6);
    assertEquals( expectedY, c.moduleTranslations[0].getY(), 1e-6);
    // Front Right: (+x, -y)
    assertEquals( expectedX, c.moduleTranslations[1].getX(), 1e-6);
    assertEquals(-expectedY, c.moduleTranslations[1].getY(), 1e-6);
    // Back Left: (-x, +y)
    assertEquals(-expectedX, c.moduleTranslations[2].getX(), 1e-6);
    assertEquals( expectedY, c.moduleTranslations[2].getY(), 1e-6);
    // Back Right: (-x, -y)
    assertEquals(-expectedX, c.moduleTranslations[3].getX(), 1e-6);
    assertEquals(-expectedY, c.moduleTranslations[3].getY(), 1e-6);
  }

  @Test
  void zeroTrackWidthThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().trackWidthMeters(0).build());
  }

  @Test
  void negativeWheelBaseThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().wheelBaseMeters(-0.1).build());
  }

  @Test
  void zeroWheelRadiusThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().wheelRadiusMeters(0).build());
  }

  @Test
  void zeroMaxSpeedThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().maxLinearSpeedMps(0).build());
  }

  @Test
  void nullModuleTypeThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().moduleType(null).build());
  }

  @Test
  void nullGyroTypeThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().gyroType(null).build());
  }

  @Test
  void defaultBuilderIsValid() {
    // Default builder should produce a working sim config out of the box
    assertDoesNotThrow(() -> new SwerveConstants.Builder().build());
  }

  // --- Physics simulation fields ---

  @Test
  void physicsFieldsHaveCorrectDefaults() {
    SwerveConstants c = new SwerveConstants.Builder().build();
    assertEquals(45.0,  c.robotMassKg,        1e-6, "default robot mass");
    assertEquals(0.76,  c.bumperLengthMeters,  1e-6, "default bumper length");
    assertEquals(0.76,  c.bumperWidthMeters,   1e-6, "default bumper width");
  }

  @Test
  void customPhysicsFieldsAreStored() {
    SwerveConstants c = validBuilder()
        .robotMassKg(60.0)
        .bumperLengthMeters(0.9)
        .bumperWidthMeters(0.85)
        .build();
    assertEquals(60.0,  c.robotMassKg,        1e-6);
    assertEquals(0.9,   c.bumperLengthMeters,  1e-6);
    assertEquals(0.85,  c.bumperWidthMeters,   1e-6);
  }

  @Test
  void robotMassBelowMinThrows() {
    // IronMaple lower bound is 10 kg
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().robotMassKg(9.9).build());
  }

  @Test
  void robotMassAboveMaxThrows() {
    // IronMaple upper bound is 80 kg (FRC weight limit with bumpers is ~68 kg)
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().robotMassKg(80.1).build());
  }

  @Test
  void bumperTooSmallThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().bumperLengthMeters(0.49).build());
  }

  @Test
  void bumperTooLargeThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().bumperWidthMeters(1.51).build());
  }
}