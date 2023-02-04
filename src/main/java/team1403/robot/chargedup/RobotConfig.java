package team1403.robot.chargedup;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
   * Swerve Constants .
   * 
   */
  public static class SwerveConfig {
    // Constants for swerve module PID regarding rotation
    public static final double kPTurning = 0.5;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 5.0;

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final SwerveDriveKinematics kDriveKinematics 
      = new SwerveDriveKinematics(
        // Front left
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
        // Front right
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
        // Back left
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
        // Back right
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

    public static final int frontLeftDriveId = 1;
    public static final int frontLeftSteerId = 2;
    public static final int frontLeftEncoderId = 1;
    public static final double frontLeftEncoderOffset = -Math.toRadians(180.263671875);
  
    public static final int frontRightDriveId = 8;
    public static final int frontRightSteerId = 3;
    public static final int frontRightEncoderId = 3;
    public static final double frontRightEncoderOffset = -Math.toRadians(267.1875);
  
    public static final int backLeftDriveId = 14;
    public static final int backLeftSteerId = 4;
    public static final int backLeftEncoderId = 2;
    public static final double backLeftEncoderOffset = -Math.toRadians(268.2421875);
  
    public static final int backRightDriveId = 2;
    public static final int backRightSteerId = 1;
    public static final int backRightEncoderId = 4;
    public static final double backRightEncoderOffset = -Math.toRadians(153.544921875);

    public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double kSteerReduction = (15.0 / 32.0) * (10.0 / 60.0);

    public static final double kSteerRelativeEncoderPositionConversionFactor = 
        2.0 * Math.PI / 2048.0 * kSteerReduction;
        
    public static final double kSteerRelativeEncoderVelocityConversionFactor = 
        kSteerRelativeEncoderPositionConversionFactor * 10.0;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kMaxSpeed = 6.0;

    public static final double maxAngularSpeed = 
        14.301625486188971;

    public static final double kVoltageSaturation = 12.0;
    public static final double kCurrentLimit = 20.0;

    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final  double kMaxAngularAccelerationRadiansPerSecondSquared = 
        Math.PI / 2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
              new TrapezoidProfile.Constraints(
                      maxAngularSpeed,
                      kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  /**
   * Configures the CAN bus. These are grouped together
   * rather than by subsystem to more easily detect conflict
   * and understand overall wiring.
   */
  public class CanBus {

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
  public class RioPorts {

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
  public class OperatorConfig {

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
  public class ExampleRail {
    
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
