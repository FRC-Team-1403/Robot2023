package team1403.lib.core;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import team1403.lib.device.Device;
import team1403.lib.device.wpi.WpiAnalogDevice;
import team1403.lib.util.CougarLogger;

/**
 * A CougarSubsystem is a subsystem of a CougarRobot.
 *
 * <p>Extends WPILib Subsystem for the CougarLib framework.
 * A subsystem is a collection of inter-dependent devices and
 * commands that coordinate them. A CougarSubsystem denotes
 * an abstraction of the physical robot where a WPILib subsystem
 * is just a means to deconflict commands that cannot execute
 * simultaneously (e.g. because they are controlling the same actuator).
 *
 * <p>Therefore a CougarSubsystem can be composed from multiple
 * WPILib Subsystem's in practice (if it has different cliques of
 * interdependent components that are independent of other cliques)
 * but this is not conveyed through the abstraction. It would be an
 * implementation detail within the subsystem.
 *
 * <p>Note that CougarSubsystem supports a void periodic() method
 * which will be called on every event loop iteration should you
 * choose to override it in subclasses.
 */
public abstract class CougarSubsystem extends SubsystemBase {
  /**
   * Constructor.
   *
   * @param name The name of the subsystem should be unique within the robot.
   * @param injectedParameters The CougarLib parameters injected into this
   *                           runtime.
   */
  public CougarSubsystem(String name,
                         CougarLibInjectedParameters injectedParameters) {
    setName(name);
    setSubsystem(name);
    m_logger = CougarLogger.getChildLogger(
        injectedParameters.getRobotLogger(), name);
  }

  /**
   * Informs the subsystem that it is managing the given device.
   *
   * @param shortName The name of the device unique within the subsystem.
   *
   * @param device The device owned by the subsystem. If it is Sendable
   *               then the SmartDashboard will be informed of it being
   *               a child of the subsystem.
   */
  public void addDevice(String shortName, Device device) {
    if (device instanceof Sendable) {
      super.addChild(shortName, (Sendable)device);

      // Group this sendable (device) with the subsystem.
      // Otherwise it will lump into the "Ungrouped" group.
      SendableRegistry.setName((Sendable)device, getName(), shortName);
    }
  }

  /**
   * Returns subsystem logger.
   *
   * @return logger bound in constructor.
   */
  public final CougarLogger getLogger() {
    return m_logger;
  }

  /**
   * Convienence function to log TRACE messages.
   *
   * @param format The message format string.
   * @param args The args for the format string.
   */
  public void tracef(String format, Object... args) {
    m_logger.tracef(format, args);
  }

  /**
   * Convienence function to log DEBUG messages.
   *
   * @param format The message format string.
   * @param args The args for the format string.
   */
  public void debugf(String format, Object... args) {
    m_logger.debugf(format, args);
  }

  final CougarLogger m_logger;
}
