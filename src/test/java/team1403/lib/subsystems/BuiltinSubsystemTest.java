package team1403.lib.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import java.util.Arrays;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.CougarAccelerometer;
import team1403.lib.subsystems.test.FakeBuiltinSubsystemParts;


/**
 * Test fixture for a BuiltinSubsystem.
 *
 * <p>This test is public to share the FakeParts with the CougarImplTest
 * for convenience.
 */
class BuiltinSubsystemTest {
  @Test
  void testConstructor() {
    var parts = new FakeBuiltinSubsystemParts();
    var parameters = new CougarLibInjectedParameters.Builder()
        .deviceFactory(parts.deviceFactory)
        .build();

    var builtin = new BuiltinSubsystem(parameters, parts.logger);
    assertEquals("BuiltinDevices", builtin.getName());

    var factoryCalls = parts.deviceFactory.getCalls();
    assertEquals(0, parts.deviceFactory.getRemainingDevices().size());
    assertEquals(Arrays.asList("BuiltinDevices.Accelerometer",
                               CougarAccelerometer.Range.k4G),
                 factoryCalls.get("BuiltinDevices.Accelerometer"));
    assertEquals(Arrays.asList("BuiltinDevices.Pdp", parts.logger),
                 factoryCalls.get("BuiltinDevices.Pdp"));

    assertSame(builtin.getPowerDistributor(), parts.powerDistributor);
    assertSame(builtin.getAccelerometer(), parts.accelerometer);
  }

  @Test
  void reportMetrics() {
    var parts = new FakeBuiltinSubsystemParts();
    var parameters = new CougarLibInjectedParameters.Builder()
        .deviceFactory(parts.deviceFactory)
        .build();

    final double kVolts = 12.3;
    final double kCelsius = 54.3;
    final double kAmps = 2.1;
    parts.powerDistributor.setVoltage(kVolts);
    parts.powerDistributor.setCelsius(kCelsius);
    parts.powerDistributor.setAmps(kAmps);

    var builtin = new BuiltinSubsystem(parameters, parts.logger);
    builtin.periodic();

    assertEquals(kVolts,
                 SmartDashboard.getEntry("BuiltinDevices.Pdp.Volts").getDouble(0));
    assertEquals(kAmps,
                 SmartDashboard.getEntry("BuiltinDevices.Pdp.Amps").getDouble(0));
    assertEquals(kCelsius,
                 SmartDashboard.getEntry("BuiltinDevices.Pdp.Temp").getDouble(0));
  }
}
