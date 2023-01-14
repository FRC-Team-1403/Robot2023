package team1403.robot.__replaceme__.examplerail;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import org.junit.jupiter.api.Test;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.test.FakeCurrentSensor;
import team1403.lib.device.test.FakeEncoder;
import team1403.lib.device.test.FakeMotorController;
import team1403.lib.device.test.MappedDeviceFactory;
import team1403.lib.device.virtual.ManualLimitSwitch;
import team1403.lib.util.CougarLogger;
import team1403.robot.__replaceme__.RobotConfig;


/**
 * Test fixture for ExampleRail subsystem.
 *
 * <p>This test is public to share the FakeParts with the CougarImplTest
 * for convenience.
 */
@SuppressWarnings("PMD.JUnit5TestShouldBePackagePrivate")
public class ExampleRailTest {
  static final double kMinSpeed = 10.0;
  static final double kSpeed = 12.34;
  static final int kFrontPort = 11;
  static final int kBackPort = 22;
  static final int kMotorChannel = 33;

  /**
   * Returns config with custom overrides for test to confirm they are used.
   */
  RobotConfig makeConfig() {
    var config = new RobotConfig();
    var railConfig = config.exampleRail;

    // Override all the config to confirm we're using it.
    railConfig.motorInverted = true;
    railConfig.minSpeed = kMinSpeed;
    railConfig.motorSpeed = kSpeed;
    config.canBus.exampleRailMotor = kMotorChannel;
    config.ports.exampleRailForwardLimitSwitch = kFrontPort;
    config.ports.exampleRailReverseLimitSwitch = kBackPort;

    return config;
  }

  @Test
  void testConstructor() {
    var parts = new FakeParts();
    var parameters = new CougarLibInjectedParameters.Builder()
      .deviceFactory(parts.deviceFactory)
      .build();

    var rail = new ExampleRail(parameters, makeConfig());
    assertEquals("Rail", rail.getName());

    var factoryCalls = parts.deviceFactory.getCalls();
    assertEquals(0, parts.deviceFactory.getRemainingDevices().size());
    assertEquals(Arrays.asList("Rail.Front", Integer.valueOf(kFrontPort)),
                  factoryCalls.get("Rail.Front"));
    assertEquals(Arrays.asList("Rail.Back", Integer.valueOf(kBackPort)),
                 factoryCalls.get("Rail.Back"));
    assertEquals(Arrays.asList("Rail.Motor", Integer.valueOf(kMotorChannel),
                               rail.getLogger()),
                 factoryCalls.get("Rail.Motor"));
  }

  @SuppressWarnings("PMD.AvoidInstantiatingObjectsInLoops")
  @Test
  void testMotorConfig() {
    for (var inverted : Arrays.asList(true, false)) {
      var parts = new FakeParts();
      var parameters = new CougarLibInjectedParameters.Builder()
        .deviceFactory(parts.deviceFactory)
        .build();
      var robotConfig = makeConfig();
      robotConfig.exampleRail.motorInverted = inverted;

      // Just look at side effect on the motor of instantiating the rail.
      // Did the constructor mark it inverted or not?
      new ExampleRail(parameters, robotConfig);
      assertEquals(inverted, parts.fakeMotor.getInverted());
    }
  }

  @Test
  void testControl() {
    var parts = new FakeParts();
    var parameters = new CougarLibInjectedParameters.Builder()
      .deviceFactory(parts.deviceFactory)
      .build();

    var rail = new ExampleRail(parameters, makeConfig());
    assertFalse(rail.isAtFront());
    assertFalse(rail.isAtBack());

    parts.frontLimitSwitch.setTriggered(true);
    assertTrue(rail.isAtFront());
    assertFalse(rail.isAtBack());
    parts.frontLimitSwitch.setTriggered(false);

    parts.backLimitSwitch.setTriggered(true);
    assertFalse(rail.isAtFront());
    assertTrue(rail.isAtBack());
    parts.backLimitSwitch.setTriggered(false);

    rail.setSpeed(kMinSpeed);
    assertEquals(kMinSpeed, parts.fakeMotor.getSpeed());

    var fakeMotor = parts.fakeMotor;
    var numStopMotorCalls = fakeMotor.countStopMotorCalls();
    rail.setSpeed(-kMinSpeed);
    assertEquals(numStopMotorCalls, fakeMotor.countStopMotorCalls());
    assertEquals(-kMinSpeed, fakeMotor.getSpeed());

    rail.setSpeed(0.1 - kMinSpeed);
    assertEquals(numStopMotorCalls + 1, fakeMotor.countStopMotorCalls());
    assertEquals(0, fakeMotor.getSpeed());

    rail.setSpeed(kMinSpeed - 0.1);
    assertEquals(numStopMotorCalls + 2, fakeMotor.countStopMotorCalls());
    assertEquals(0, fakeMotor.getSpeed());
  }

  /**
   * Helper class providing fake parts for assembling an ExampleRail for testing.
   */
  @SuppressWarnings("PMD.DataClass")
  public static class FakeParts {
    /**
     * Construct fake devices and factory for assembling a faked Rail.
     */
    public FakeParts() {
      frontLimitSwitch = new ManualLimitSwitch("Rail.Front");
      backLimitSwitch = new ManualLimitSwitch("Rail.Back");
      fakeCurrentSensor = new FakeCurrentSensor("Rail.CurrentSensor");
      fakeEncoder = new FakeEncoder("Rail.Encoder", kTicksPerRevolution);
      fakeMotor = new FakeMotorController("Rail.Motor", logger,
                                          fakeEncoder, fakeCurrentSensor);

      deviceFactory = new MappedDeviceFactory();
      deviceFactory.putLimitSwitch(frontLimitSwitch);
      deviceFactory.putLimitSwitch(backLimitSwitch);
      deviceFactory.putMotorController(fakeMotor);
    }

    public CougarLogger logger = CougarLogger.getCougarLogger("Rail");
    public ManualLimitSwitch frontLimitSwitch;
    public ManualLimitSwitch backLimitSwitch;
    public FakeCurrentSensor fakeCurrentSensor;
    public FakeEncoder fakeEncoder;
    public FakeMotorController fakeMotor;
    public MappedDeviceFactory deviceFactory;

    public static final double kTicksPerRevolution = 1024.0;
  }
}
