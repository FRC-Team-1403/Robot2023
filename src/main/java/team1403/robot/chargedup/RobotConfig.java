package team1403.robot.chargedup;

/**
 * This class holds attributes for the robot configuration.
 *
 * <p>The RobotConfig is broken out into different areas,
 * each of which is captured in a class for that area. Each
 * subsystem has its own independent config.
 *
 * <p>The "electrical" configs are treated separate and independent
 * to make it easier to see how the robot should be wired and see
 * any conflicts since these ports specify their config together.
 */
public final class RobotConfig {
  /**
   * Configures the CAN bus. These are grouped together
   * rather than by subsystem to more easily detect conflict
   * and understand overall wiring.
   */
  public static class CanBus {
    /**
     * The can bus port for the rail motor if it is a TalonSRX.
     *
     * <p>Should be -1 if exampleRailSparkMotor was set.
     */
    public static final int exampleRailMotor = 10;  // talon

    /**
     * The can bus port for the rail motor if it is a SparkMax.
     *
     * <p>Should be -1 if exampleRailMotor was set.
     */
    public static final int exampleRailSparkMotor = -1;
  }

  /**
   * Ports on the RoboRIO.
   */
  public static class RioPorts {
    /**
     * The rio port that the forward limit switch uses.
     */
    public static final int exampleRailForwardLimitSwitch = 1;

    /**
     * This switch is optional. Set to port -1 if it is not available.
     */
    public static final int exampleRailReverseLimitSwitch = 2;
  }

  /**
   * Config parameters for tuning the operator interface.
   */
  public static class OperatorConfig {
    /**
     * The joystick port for the driver's controller.
     */
    public static final int pilotPort = 1;

    /**
     * Encoder ticks from center still considered close enough to be at center.
     */
    public static final double seekCenterTolerance = 10.0;
  }

  /**
   * exampleRail subsystem configuration.
   *
   * <p>Encapsulates the parameters controlling the rail subsystem behavior.
   * The device wiring parameters are specified with their respective bus's
   * configuration.
   *
   */
  public static class ExampleRail {
    /**
     * True if the motor is inverted.
     */
    public static final boolean motorInverted = false;

    /**
     * The default motor speed for commands.
     */
    public static final double motorSpeed = 0.75;

    /**
     * The minimum motor speed to move.
     *
     * <p>Anything less will be considered stopping.
     */
    public static final double minSpeed = 0.01;

    /**
     * Assumed length of rail in encoder ticks.
     *
     * <p>This is used if we have no exampleRailReverseLimitSwitch.
     * In that case we will use a virtual one that will trigger when
     * the encoder is this far from the front switch.
     */
    public static final long virtualBackLimitSwitchTicks = 2000;
  }
}
