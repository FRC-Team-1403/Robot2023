package team1403.lib.device.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import org.junit.jupiter.api.Test;

import team1403.lib.device.LimitSwitch;
import team1403.lib.device.MotorController;
import team1403.lib.device.NoSuchDeviceError;
import team1403.lib.device.virtual.ManualLimitSwitch;
import team1403.lib.util.CougarLogger;


/**
 * MappedDeviceFactory test fixture.
 *
 * <p>Verify that the MappedDeviceFactory we are relying on to test things
 * is itself working as intended.
 */
class MappedDeviceFactoryTest {
  static final String kLimitSwitchName = "TestLimitSwitch";
  static final String kTalonName = "TestTalon";
  static final String kVictorName = "TestVictor";
  static final int kLimitSwitchPort = 111;
  static final int kTalonPort = 222;
  static final int kVictorPort = 333;

  final CougarLogger m_deviceLogger = CougarLogger.getLoggerForClass(
      MappedDeviceFactoryTest.class);
  final LimitSwitch m_limitSwitch = new ManualLimitSwitch(kLimitSwitchName);
  final MotorController m_talon = new FakeMotorController(kTalonName, m_deviceLogger);
  final MotorController m_victor = new FakeMotorController(kVictorName, m_deviceLogger);

  @Test
  void deviceLookupTest() {
    var factory = new MappedDeviceFactory();
    factory.putLimitSwitch(m_limitSwitch);
    factory.putMotorController(m_victor);
    factory.putMotorController(m_talon);
    assertEquals(3, factory.getRemainingDevices().size());
    assertTrue(factory.remainingDeviceNamesToString().contains(kVictorName));

    final var gotSwitch = factory.makeLimitSwitch(kLimitSwitchName, kLimitSwitchPort);
    assertSame(m_limitSwitch, gotSwitch);

    final var gotTalon = factory.makeTalon(kTalonName, kTalonPort, m_deviceLogger);
    assertSame(gotTalon, m_talon);

    final var gotVictor = factory.makeVictorSpx(kVictorName, kVictorPort, m_deviceLogger);
    assertSame(gotVictor, m_victor);

    assertEquals(0, factory.getRemainingDevices().size());
    assertEquals("", factory.remainingDeviceNamesToString());
    assertEquals(3, factory.getCalls().size());

    assertEquals(factory.getCalls().get(kLimitSwitchName),
                 Arrays.asList(kLimitSwitchName,
                               Integer.valueOf(kLimitSwitchPort)));
    assertEquals(factory.getCalls().get(kVictorName),
                 Arrays.asList(kVictorName, Integer.valueOf(kVictorPort),
                               m_deviceLogger));
    assertEquals(factory.getCalls().get(kTalonName),
                 Arrays.asList(kTalonName, Integer.valueOf(kTalonPort),
                               m_deviceLogger));
  }

  @Test
  void invalidCalls() {
    var factory = new MappedDeviceFactory();
    factory.putLimitSwitch(m_limitSwitch);

    assertThrows(IllegalArgumentException.class, () -> {
      factory.putLimitSwitch(m_limitSwitch);
    });

    var got = factory.makeLimitSwitch(kLimitSwitchName, kLimitSwitchPort);
    assertSame(m_limitSwitch, got);
    assertThrows(NoSuchDeviceError.class, () -> {
      factory.makeLimitSwitch(kLimitSwitchName, kLimitSwitchPort);
    });

    factory.putMotorController(m_talon);
    assertThrows(ClassCastException.class, () -> {
      factory.makeLimitSwitch(kTalonName, kLimitSwitchPort);
    });

    // Consumed device.
    // This isn't intentional but surviving this exception is not expected
    // since it indicates a fundamental programming error that would
    // arise during startup.
    assertEquals(0, factory.getRemainingDevices().size());
  }
}
