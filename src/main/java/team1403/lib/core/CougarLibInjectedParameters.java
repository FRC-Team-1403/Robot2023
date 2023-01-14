package team1403.lib.core;

import team1403.lib.device.DeviceFactory;
import team1403.lib.device.wpi.RealDeviceFactory;
import team1403.lib.util.Clock;
import team1403.lib.util.CougarLogger;
import team1403.lib.util.WpiClock;


/**
 * Standard parameters for a CougarRobot.
 *
 * <p>These are environmental parameters injected in so robots
 * can be constructed differently for testing, simulation, and
 * "real" minimizing the need of the implementation to distinguish
 * among them. This simplifies things and makes for more meaningful
 * testing since the code under test is the same as the live code.
 */
public class CougarLibInjectedParameters {
  /**
   * The builder acts as the factory so that the parameters are immutable.
   *
   * <p>The values can change within the builder, but once the builder
   * creates the parameter instance that the robot will use, these paramters
   * become final.
   */
  public static class Builder {
    /**
     * Constructor uses default values.
     */
    public Builder() {
      // empty
    }

    /**
     * Constructor uses supplied values as its defaults.
     *
     * @param defaultValues The default parameter values.
     */
    public Builder(CougarLibInjectedParameters defaultValues) {
      m_clock = defaultValues.getClock();
      m_robotLogger = defaultValues.getRobotLogger();
      m_deviceFactory = defaultValues.getDeviceFactory();
    }

    /**
     * Create a parameters instance using this configuration.
     *
     * @return new immutable CougarLibInjectredParameters instance.
     */
    public CougarLibInjectedParameters build() {
      return new CougarLibInjectedParameters(this);
    }

    /**
     * Change the clock.
     *
     * @param clock The clock the parameters will specify.
     * @return this builder
     */
    public Builder clock(Clock clock) {
      m_clock = clock;
      return this;
    }

    /**
     * Change the logger.
     *
     * @param logger The root robot logger the parameters will specify.
     * @return this builder
     */
    public Builder robotLogger(CougarLogger logger) {
      m_robotLogger = logger;
      return this;
    }

    /**
     * Change the device factory.
     *
     * @param factory The device factory to use.
     * @return this builder.
     */
    public Builder deviceFactory(DeviceFactory factory) {
      m_deviceFactory = factory;
      return this;
    }

    private Clock m_clock = WpiClock.instance();
    private CougarLogger m_robotLogger
        = CougarLogger.getCougarLogger("team1403.Robot");
    private DeviceFactory m_deviceFactory = new RealDeviceFactory();
  }  // end Builder

  /**
   * Returns the clock to use.
   *
   * @return the configured clock
   */
  public Clock getClock() {
    return m_clock;
  }

  /**
   * Returns the logger acting as the root logger for the robot.
   *
   * @return the configured logger
   */
  public CougarLogger getRobotLogger() {
    return m_robotLogger;
  }

  /**
   * Return the factory for creating new devices.
   *
   * @return factory to use when constructing the robot runtime.
   */
  public DeviceFactory getDeviceFactory() {
    return m_deviceFactory;
  }

  /**
   * Constructor.
   *
   * @param builder Specifies the parameters to use.
   */
  protected CougarLibInjectedParameters(Builder builder) {
    m_clock = builder.m_clock;
    m_robotLogger = builder.m_robotLogger;
    m_deviceFactory = builder.m_deviceFactory;
  }

  private final Clock m_clock;
  private final CougarLogger m_robotLogger;
  private final DeviceFactory m_deviceFactory;
}
