package team1403.robot.chargedup;

import team1403.lib.util.Dimension;

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

    public static final int wheelIntakeMotor = 1;
    
    public static final int telescopicArmMotor = 1;

    public static final int leftAngledArmMotor = 2; 

    public static final int rightAngledArmMotor = 3;

    public static final int wristMotor = 4;

    public static final int frontArmSwitch = 1;

    public static final int backArmSwitch = 2;
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
    public static final int frontArmSwitch = 0;

    public static final int backArmSwitch = 0;

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

  /**
   * class Arm, sets constant values for PID for Arm.java.
   */
  public static class Arm {
    public static final int kP = 0; //constant for Proportional
    public static final int kI = 0; //constant for Integral
    public static final int kD = 0; //constant for Derivative

    public static final double kArmConversionFactor = 1;
    public static final double kWristConversionFactor = 2;
    public static final double kArmLengthConversionFactor = 3;
    public static final double kWheelIntakeConversionFactor = 4;

    double wristAngle = 0;

    public static final double kMaxArmRotation = 270;
    public static final double kMinArmRotation = 0;
    public static final double kMaxWristRotation = 90;
    public static final double kMinWristRotation = 0;
    public static final double kMinArmExtension = 5;
    public static final double kMaxArmExtension = 15;

    public static final Dimension robotDimensions = new Dimension(0, 0, 0);

    public static final Dimension wristDimensions = new Dimension(0, 0, 0);

    public static final double kMaxAmperage = 0;

    public static final double kMaxArmLengthOffset = 0;
    public static final double kAngleToMeters = 0;

    /**
     * This is the angular threshold to determine at what
     * point the maximum extenstion of the arm should be
     * limited by the arm hitting the ground.
     */
    public static final double kMaxGroundArmLengthThreshold = 0;
  }
}
