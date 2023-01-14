package team1403.lib.device.virtual;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

import team1403.lib.device.LimitSwitch;

/**
 * Test fixture for multiple simple virtual devices.
 */
class SimpleVirtualDeviceTest {
  static final NetworkTableInstance tableInstance = NetworkTableInstance.create();

  /**
   * Helper function for testing limit switches.
   *
   * @param device The limit switch to test.
   * @parma proxy A proxy to the switch where if we change state here,
   *        the device should change so we can control the test.
   */
  void limitSwitchTestHelper(LimitSwitch device, ManualLimitSwitch proxy) {
    final String name = device.getName();

    var table = tableInstance.getTable(name);
    try (var builder = new SendableBuilderImpl();) {
      builder.setTable(table);
      ((Sendable)device).initSendable(builder);
      builder.update();
      assertEquals("LimitSwitch", table.getEntry(".type").getString("NotFound"));
      var triggeredEntry = table.getEntry("Triggered");

      assertFalse(device.isTriggered());
      assertFalse(triggeredEntry.getBoolean(true));

      proxy.setTriggered(true);
      assertTrue(device.isTriggered());
      assertFalse(triggeredEntry.getBoolean(true));
      builder.update();
      assertTrue(triggeredEntry.getBoolean(false));

      proxy.setTriggered(false);
      assertFalse(device.isTriggered());
      assertTrue(triggeredEntry.getBoolean(false));
      builder.update();
      assertFalse(triggeredEntry.getBoolean(true));
    }
  }

  @Test
  void testManualLimitSwitch() {
    final String name = "TestManualLimitSwitch";
    var device = new ManualLimitSwitch(name);
    assertEquals(name, device.getName());

    limitSwitchTestHelper(device, device);

    // Supposedly the sendable builder allows us to go bidrectional
    // and should be able to update our trigger by modifying the table,
    // But I cannot figure out to get this to work so we'll leave it out
    // for now.
  }

  @Test
  void testLimitSwitchImpl() {
    var manualLimitSwitch = new ManualLimitSwitch("helper");
    final String name = "TestLimitSwitchImpl";

    // Our device will use the manualSwitch isTriggered as it's callback
    // so that we can change the values in the manual switch because otherwise
    // java lambdas require final values so we cant change them in the test.
    var device = new LimitSwitchImpl(name, manualLimitSwitch::isTriggered);
    assertEquals(name, device.getName());

    limitSwitchTestHelper(device, manualLimitSwitch);
  }
}
