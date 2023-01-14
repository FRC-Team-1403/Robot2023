package team1403.lib.core;

import edu.wpi.first.wpilibj2.command.Command;

import team1403.lib.util.CougarLogger;


/**
 * Base class for CougarRobots.
 *
 * <p>In practice, this is subclassed into a specialized class for a given robot.
 * That specialized class will introduce the subsystems and command bindings
 * for that robot where the base class will contain the boilerplate code and
 * standard features and framwork.
 */
public class CougarRobot {
  /**
   * Denotes the current operating mode.
   */
  public enum Mode {
    /**
     * The robot is offline and not yet communicating.
     */
    OFFLINE,

    /**
     * The robot is disabled, meaning the actuators are disabled.
     *
     * <p>It is still alive, sensing, and communicating.
     */
    DISABLED,

    /**
     * The robot is in autonomous mode.
     *
     * <p>It is enabled but not receiving input from the DriverStation.
     */
    AUTONOMOUS,

    /**
     * The robot is fully enabled.
     *
     * <p>This is the normal use case where operators can override control
     * and the robot's mission is to play the game it was designed for.
     */
    TELEOP,

    /**
     * The robot is in test mode.
     *
     * <p>This is similar to TELEOP except the mission is to "test" the robot.
     */
    TEST,
  }

  static CougarLibInjectedParameters.Builder newParameterBuilder() {
    return new CougarLibInjectedParameters.Builder();
  }

  static CougarLibInjectedParameters.Builder newParameterBuilder(
      CougarLibInjectedParameters defaultValues) {
    return new CougarLibInjectedParameters.Builder(defaultValues);
  }

  /**
   * Constructor.
   *
   * @param parameters Provide framework configuration parameters.
   *        These are not intended to be custom robot configuration
   *        parameters (like wiring and speed limits) rather they
   *        are things like the Clock to use.
   */
  public CougarRobot(CougarLibInjectedParameters parameters) {
    m_parameters = parameters;
    m_logger = m_parameters.getRobotLogger();
    m_logger.infof("Constructing CougarRobot as '%s'", getClass().getName());
  }

  /**
   * Returns the parameters bound by the constructor.
   *
   * @return framework parameters
   */
  public final CougarLibInjectedParameters getInjectedParameters() {
    return m_parameters;
  }

  /**
   * Informs the robot that it is in a new operating mode.
   *
   * @param mode The new mode.
   *
   * @see startMode
   */
  public final void changeMode(Mode mode) {
    m_mode = mode;
    startMode(mode);
  }

  /**
   * Returns the current operating mode.
   *
   * @return The current operating mode.
   */
  public final Mode getMode() {
    return m_mode;
  }

  /**
   * Returns the autonomous command.
   *
   * <p>The autonomous command is treated special because the
   * WpiLibRobotAdaptor will run it for us so needs to know what it is.
   *
   * @return The command to run in autonomous mode.
   */
  public Command getAutonomousCommand() {
    return null;
  }

  /**
   * Overridable hook called by {@link #startMode}.
   *
   * <p>Can be overriden if custom robot wants to react in specific ways.
   *
   * @param mode The current robot mode that has just started.
   */
  protected void startMode(Mode mode) {
    // empty;
  }

  /**
   * Write a trace message to the logger.
   *
   * @param format The String.format string
   * @param args The args to populate the String.format string.
   */
  protected void tracef(String format, Object... args) {
    m_logger.tracef(format, args);
  }

  private final CougarLibInjectedParameters m_parameters;
  private final CougarLogger m_logger;
  private Mode m_mode = Mode.OFFLINE;
}
