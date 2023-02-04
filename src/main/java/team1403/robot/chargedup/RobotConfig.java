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
    
    public int telescopicArmMotor = 1;

    public int leftAngledArmMotor = 2; 

    public int rightAngledArmMotor = 3;

    public int wristMotor = 4;

    public int frontArmSwitch = 1;

    public int backArmSwitch = 2;
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
    public int exampleRailForwardLimitSwitch = 1;

    /**
     * This switch is optional. Set to port -1 if it is not available.
     */
    public int exampleRailReverseLimitSwitch = 2;
  }

  /**
   * Config parameters for tuning the operator interface.
   */
  public static class OperatorConfig {
    /**
     * The joystick port for the driver's controller.
     */
    public int pilotPort = 1;

    /**
     * Encoder ticks from center still considered close enough to be at center.
     */
    public double seekCenterTolerance = 10.0;
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
    public boolean motorInverted = false;

    /**
     * The default motor speed for commands.
     */
    public double motorSpeed = 0.75;

    /**
     * The minimum motor speed to move.
     *
     * <p>Anything less will be considered stopping.
     */
    public double minSpeed = 0.01;

    /**
     * Assumed length of rail in encoder ticks.
     *
     * <p>This is used if we have no exampleRailReverseLimitSwitch.
     * In that case we will use a virtual one that will trigger when
     * the encoder is this far from the front switch.
     */
    public long virtualBackLimitSwitchTicks = 2000;
  }

  /**
   * class Arm, sets constant values for PID for Arm.java.
   */
  public static class Arm {
    public int kP = 0; //constant for Proportional
    public int kI = 0; //constant for Integral
    public int kD = 0; //constant for Derivative

    double wristAngle = 0;

    public double kMaxArmRotation = 270;
    public double kMinArmRotation = 0;
    public double kMaxWristRotation = 90;
    public double kMinWristRotation = 0;
    public double kMinArmExtension = 5;
    public double kMaxArmExtension = 15;

    public Dimension robotDimensions = new Dimension(0, 0, 0);

    public Dimension wristDimensions = new Dimension(0, 0, 0);

    public double kMaxAmperage = 0;

    public double kMaxArmLengthOffset = 0;
    public double kAngleToMeters = 0;

    /**
     * This is the angular threshold to determine at what
     * point the maximum extenstion of the arm should be
     * limited by the arm hitting the ground
     */
    public double kMaxGroundArmLengthThreshold = 0;
  }
  
  /**
   * class switch, to define type Switch for Limit switch.
   */
  public static class Switch {

  }

  // These are the actual configuration attributes.
  // Each independent aspect of config has its own type
  // so they are scoped to where they are needed and relevant.

  /**
   * The CAN bus configuration.
   */
  public CanBus canBus = new CanBus();

  /**
   * The port allocation on the RoboRIO.
   */
  public RioPorts ports = new RioPorts();

  /**
   * Configuration related to the operator interface.
   */
  public OperatorConfig operator = new OperatorConfig();

  /**
   * Configuration for the ExampleRail subsystem.
   */
  public ExampleRail exampleRail = new ExampleRail();

  /**
   * Configuration for the Arm subsystem.
   */
  public Arm arm = new Arm();
}
