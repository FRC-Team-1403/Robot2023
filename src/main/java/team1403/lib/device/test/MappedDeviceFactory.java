package team1403.lib.device.test;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;

import team1403.lib.device.AnalogDevice;
import team1403.lib.device.CougarAccelerometer;
import team1403.lib.device.CougarDoubleSolenoid;
import team1403.lib.device.Device;
import team1403.lib.device.DeviceFactory;
import team1403.lib.device.LimitSwitch;
import team1403.lib.device.MotorController;
import team1403.lib.device.NoSuchDeviceError;
import team1403.lib.device.PowerDistributor;
import team1403.lib.device.virtual.Limelight;
import team1403.lib.util.CougarLogger;


/**
 * A device factory for testing.
 *
 * <p>A MappedDeviceFactory is pre-populated with
 * the devices to return when asked for by name. It
 * is meant for testing. Devices are removed from the
 * map as they are returned since names are supposed
 * to be unique.
 */
@SuppressWarnings({"PMD.UseConcurrentHashMap", "PMD.TooManyMethods"})
public class MappedDeviceFactory implements DeviceFactory {
  /**
   * Update the factory with all the components of the other factory.
   *
   * @param other The other factory supplying additional components
   *              must have disjoint components in it.
   *
   * @throws IllegalArgumentException if other factory has keys in use.
   */
  public void update(MappedDeviceFactory other) {
    for (Device device : other.m_deviceMap.values()) {
      putDevice(device);
    }
  }

  /**
   * Adds a MotorController.
   *
   * @param controller The MotorController to return when asked for.
   */
  public void putMotorController(MotorController controller) {
    putDevice(controller);
  }

  /**
   * Adds a limit switch.
   *
   * @param limitSwitch The limit switch to return when asked for.
   */
  public void putLimitSwitch(LimitSwitch limitSwitch) {
    putDevice(limitSwitch);
  }

  /**
   * Adds a limelight.
   *
   * @param limelight The limelight to return when asked for.
   */
  public void putLimelight(Limelight limelight) {
    putDevice(limelight);
  }

  /**
   * Populates the factory with a device.
   *
   * @param device The device must have a unique name.
   */
  public void putDevice(Device device) {
    final String name = device.getName();
    if (m_deviceMap.containsKey(name)) {
      throw new IllegalArgumentException(
          String.format("Device '%'s already exists", name));
    }
    m_deviceMap.put(name, device);
  }

  /**
   * Lookup and return a device.
   *
   * <p>The device will removed from the factory and return
   * {@code null} next time.
   *
   * @param name The desired device name.
   * @return The device previously added with {@link #putDevice} or variant.
   *
   * @throws NoSuchDeviceError if the name is not (or no longer)
   *                           known to the factory.
   */
  public Device takeDevice(String name) {
    if (!m_deviceMap.containsKey(name)) {
      throw new NoSuchDeviceError(
          String.format("Unexpected device '%s'", name));
    }
    return m_deviceMap.remove(name);
  }

  /**
   * Returns the remaining devices as a human-readable string.
   *
   * <p>This is to support writing tests that fail if there are devices left
   * over. The test can match this string against "" and show what was left
   * over as it fails.
   *
   * @return String form of enumerated devices.
   */
  public String remainingDeviceNamesToString() {
    if (m_deviceMap.isEmpty()) {
      return "";
    }
    return m_deviceMap.keySet().toString();
  }

  /**
   * Return the populated devices.
   *
   * <p>This is intended to allow tests to discover what has not been asked for.
   *
   * @return Map of registered device names to their instances.
   */
  public Map<String, Device> getRemainingDevices() {
    return m_deviceMap;
  }

  /**
   * Provides the list of parameters passed when asking for devices.
   *
   * <p>This is intended to allow tests to verify the parameters given to
   * the factory when creating devices. For example the channel that the
   * device is wired to (if applicable).
   *
   * @return Map of registered device names to the parameter list of the factory
   *         method used to create the device.
   */
  public Map<String, List<Object>> getCalls() {
    return m_calls;
  }

  @Override
  public CougarAccelerometer makeBuiltinAccelerometer(
      String name, CougarAccelerometer.Range range) {
    m_calls.put(name, Arrays.asList(name, range));
    return (CougarAccelerometer)takeDevice(name);
  }

  @Override
  public PowerDistributor makePowerDistributor(String name, CougarLogger logger) {
    m_calls.put(name, Arrays.asList(name, logger));
    return (PowerDistributor)takeDevice(name);
  }

  @Override
  public MotorController makeBrushlessCanSparkMax(
      String name, int channel, SparkMaxRelativeEncoder.Type encoderType, CougarLogger logger) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel), encoderType, logger));
    return (MotorController)takeDevice(name);
  }

  @Override
  public MotorController makeBrushedCanSparkMax(
      String name, int channel, SparkMaxRelativeEncoder.Type encoderType, CougarLogger logger) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel), encoderType, logger));
    return (MotorController)takeDevice(name);
  }

  @Override
  public MotorController makeVictorSpPwm(
      String name, int channel, CougarLogger logger) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel), logger));
    return (MotorController)takeDevice(name);    
  }

  @Override
  public MotorController makeVictorSpx(
      String name, int channel, CougarLogger logger) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel), logger));
    return (MotorController)takeDevice(name);
  }

  @Override
  public MotorController makeTalon(
      String name, int channel, CougarLogger logger) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel), logger));
    return (MotorController)takeDevice(name);
  }

  @Override
  public LimitSwitch makeLimitSwitch(String name, int channel) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel)));
    return (LimitSwitch)takeDevice(name);
  }

  public Limelight makeLimelight(String name, NetworkTable table) {
    m_calls.put(name, Arrays.asList(name, table));
    return (Limelight)takeDevice(name);
  }

  @Override
  public CougarDoubleSolenoid makeDoubleSolenoid(
      String name, int forwardChannel, 
      int reverseChannel, CougarLogger logger) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(forwardChannel), 
                Integer.valueOf(reverseChannel), logger));
    return (CougarDoubleSolenoid)takeDevice(name);
  }

  @Override
  public AnalogDevice makeAnalogDevice(String name, int channel) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel)));
    return (AnalogDevice)takeDevice(name);
  }
  
  private final Map<String, Device> m_deviceMap = new HashMap<>();
  private final Map<String, List<Object>> m_calls = new HashMap<>();
}
