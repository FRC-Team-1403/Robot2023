package team1403.robot.chargedup;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
public class RobotConfig {

  /**
   * Variables to used by all subsystems.
   */
  public static final Dimension robotDimensions = new Dimension(0, 0, 0);

  public static double kRobotHeight = 32;
  public static double kHeightFromGround = 33.72326;
  public static double kGroundToTopOfFrame = 1.72326;
  public static double kFrameHeight = 2;

  /**
   * Swerve Constants.
   * 
   */
  public static class SwerveConfig {
    public static final int kEncoderResetIterations = 500;
    public static final double kEncoderResetMaxAngularVelocity = Math.toRadians(0.5);
    public static final int kStatusFrameGeneralPeriodMs = 250;
    public static final int kCanTimeoutMs = 250;

    public static final double kPTurning = 0.5;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 5.0;

    public static final double kPAutoTurning = 0.5;
    public static final double kIAutoTurning = 0.0;
    public static final double kDAutoTurning = 5.0;

    public static final double kPTranslation = 2.0;
    public static final double kITranslation = 0.25;
    public static final double kDTranslation = 0.2;

    public static final double kTrackWidth = Units.inchesToMeters(21);
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
        // Front right
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
        // Back left
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
        // Back right
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

    public static final double frontLeftEncoderOffset = -Math.toRadians(180.263671875);
    public static final double frontRightEncoderOffset = -Math.toRadians(267.1875);
    public static final double backLeftEncoderOffset = -Math.toRadians(268.2421875);
    public static final double backRightEncoderOffset = -Math.toRadians(153.544921875);

    public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double kSteerReduction = (15.0 / 32.0) * (10.0 / 60.0);

    public static final double kSteerRelativeEncoderPositionConversionFactor = 2.0 * Math.PI 
        / 2048.0 * kSteerReduction;

    public static final double kSteerRelativeEncoderVelocityConversionFactor = 
        kSteerRelativeEncoderPositionConversionFactor * 10.0;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kMaxSpeed = 6.0;

    public static final double kMaxAngularSpeed = 14.301625486188971;

    public static final double kVoltageSaturation = 12.0;
    public static final double kCurrentLimit = 20.0;

    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(kMaxAngularSpeed, 
        kMaxAngularAccelerationRadiansPerSecondSquared);
  }

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
     * The joystick port for the operator's controller.
     */
    public static final int pilotPort = 1;

    /**
     * Encoder ticks from center still considered close enough to be at center.
     */
    public static final double seekCenterTolerance = 10.0;
  }

  /**
   * Config parameters for tuning the operator interface.
   */
  public class DriverConfig {

    /**
     * The joystick port for the operator's controller.
     */
    public static final int pilotPort = 1;

    /**
     * Encoder ticks from center still considered close enough to be at center.
     */
    public static final double seekCenterTolerance = 10.0;
  }

  /**
   * class Arm, sets constant values for PID for Arm.java.
   */
  public static class Arm {

    public static double angleHittingRobot = 0;
    public static double angleHittingGround = 0;

    public static double kPhysicalArmMaxExtension = 60.218;

    public static final int kP = 0; //constant for Proportional
    public static final int kI = 0; //constant for Integral
    public static final int kD = 0; //constant for Derivative

    public static final double kArmConversionFactor = 1;
    public static final double kWristConversionFactor = 2;
    public static final double kArmLengthConversionFactor = 3;
    public static final double kWheelIntakeConversionFactor = 4;

    public static final double kMaxArmRotation = 270;
    public static final double kMinArmRotation = 0;
    public static final double kMaxWristRotation = 90;
    public static final double kMinWristRotation = 0;
    public static final double kMinArmExtension = 5;
    public static final double kMaxArmExtension = 15;

    

    public static final Dimension wristDimensions = new Dimension(0, 0, 0);

    public static final double kMaxAmperage = 0;

    public static final double kMaxArmLengthOffset = 0;
    public static final double kAngleToMeters = 0;

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
