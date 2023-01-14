package team1403.lib.subsystems.test;

import team1403.lib.device.CougarAccelerometer;
import team1403.lib.device.test.FakeCougarAccelerometer;
import team1403.lib.device.test.FakePowerDistributor;
import team1403.lib.device.test.MappedDeviceFactory;
import team1403.lib.util.CougarLogger;

/**
 * Provides fake parts for assembling a fake BuiltinSubsystem for testing.
 */
@SuppressWarnings("PMD.DataClass")
public class FakeBuiltinSubsystemParts {
  /**
   * Construct fake devices and factory for assembling a fake BuiltinSubsystem.
   */
  public FakeBuiltinSubsystemParts() {
    accelerometer = new FakeCougarAccelerometer("BuiltinDevices.Accelerometer",
                                                CougarAccelerometer.Range.k4G);
    powerDistributor = new FakePowerDistributor("BuiltinDevices.Pdp",
                                                FakePowerDistributor.kDefaultNumCircuits);
    deviceFactory = new MappedDeviceFactory();
    deviceFactory.putDevice(accelerometer);
    deviceFactory.putDevice(powerDistributor);
  }

  /**
   * The fake accelerometer in the deviceFactory.
   */
  public FakeCougarAccelerometer accelerometer;

  /**
   * The fake PowerDistributorin the deviceFactory.
   */
  public FakePowerDistributor powerDistributor;

  /**
   * Contains the fake objects to instantiate a BuiltinSubsystem.
   */
  public MappedDeviceFactory deviceFactory;

  /**
   * The logger used by the fake devices.
   */
  public CougarLogger logger = CougarLogger.getCougarLogger("BuiltinSubsystem");
}

