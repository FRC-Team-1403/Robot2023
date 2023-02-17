package team1403.robot.chargedup;

import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import team1403.lib.util.Dimension;


/**
 * This class holds attributes for the robot configuration.
 *
 * <p>
 * The RobotConfig is broken out into different areas,
 * each of which is captured in a class for that area. Each
 * subsystem has its own independent config.
 *
 * <p>
 * The "electrical" configs are treated separate and independent
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

    public static final double kPTurning = 1.125;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0;

    public static final double kPAutoTurning = 3;
    public static final double kIAutoTurning = 0.0;
    public static final double kDAutoTurning = 0.0;

    public static final double kPTranslation = 12;
    public static final double kITranslation = 0.0;
    public static final double kDTranslation = 0.5;

    public static final double kTrackWidth = Units.inchesToMeters(21);
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final SwerveDriveKinematics kFirstOrderDriveKinematics = 
        new SwerveDriveKinematics(
            // Front left
            new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
            // Front right
            new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
            // Back left
            new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
            // Back right
            new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

    public static final double frontLeftEncoderOffset = -(4.667903537536006 + Math.PI);
    public static final double frontRightEncoderOffset = -3.109379057044195;
    public static final double backLeftEncoderOffset = -(4.663301595172349 + Math.PI);
    public static final double backRightEncoderOffset = -(5.872078456026235 + Math.PI);

    public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double kSteerReduction = (15.0 / 32.0) * (10.0 / 60.0);

    public static final double kSteerRelativeEncoderPositionConversionFactor = 2.0 * Math.PI
        * SwerveConfig.kSteerReduction;

    public static final double kSteerRelativeEncoderVelocityConversionFactor = 2.0 * Math.PI
        * SwerveConfig.kSteerReduction / 60.0;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kMaxSpeed = 14.5;

    public static final double kMaxAngularSpeed = (kMaxSpeed
        / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0)); // 39.795095397

    public static final double kVoltageSaturation = 12.0;
    public static final double kCurrentLimit = 20.0;

    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed,
        kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static class VisionConfig {
    public static AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(Arrays.asList(
      new AprilTag(1,   (new Pose3d(
        Units.inchesToMeters(610.77),
        Units.inchesToMeters(42.19),
        Units.inchesToMeters(18.22),
        new Rotation3d(0.0, 0.0, Math.PI)))),
      new AprilTag(2, new Pose3d(
        Units.inchesToMeters(610.77),
        Units.inchesToMeters(108.19),
        Units.inchesToMeters(18.22),
        new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(3,new Pose3d(
        Units.inchesToMeters(610.77),
        Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
        Units.inchesToMeters(18.22),
        new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(4, new Pose3d(
        Units.inchesToMeters(636.96),
        Units.inchesToMeters(265.74),
        Units.inchesToMeters(27.38),
        new Rotation3d(0.0, 0.0, Math.PI))), 
      new AprilTag(5, new Pose3d(
        Units.inchesToMeters(14.25),
        Units.inchesToMeters(265.74),
        Units.inchesToMeters(27.38),
        new Rotation3d())), 
      new AprilTag(6, new Pose3d(
        Units.inchesToMeters(610.77),
        Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
        Units.inchesToMeters(18.22),
        new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(7, new Pose3d(
        Units.inchesToMeters(610.77),
        Units.inchesToMeters(108.19),
        Units.inchesToMeters(18.22),
        new Rotation3d(0.0, 0.0, Math.PI))),  
      new AprilTag(8, new Pose3d(
        Units.inchesToMeters(40.45),
        Units.inchesToMeters(42.19),
        Units.inchesToMeters(18.22),
        new Rotation3d()))), Units.inchesToMeters(651.25), Units.inchesToMeters(315.5));
  }

  /**
   * Configures the CAN bus. These are grouped together
   * rather than by subsystem to more easily detect conflict
   * and understand overall wiring.
   */
  public static class CanBus {

    // Arm Ports
    public static final int wheelIntakeMotor = 3;
    public static final int telescopicArmMotor = 4;
    public static final int leftAngledArmMotor = 5;
    public static final int rightAngledArmMotor = 6;
    public static final int wristMotor = 7;
    public static final int frontArmSwitch = 9;
    public static final int backArmSwitch = 10;

    // Swerve CanBus ids
    public static final int frontLeftDriveId = 1;
    public static final int frontLeftSteerId = 15;
    public static final int frontLeftEncoderId = 3;

    public static final int frontRightDriveId = 8;
    public static final int frontRightSteerId = 7;
    public static final int frontRightEncoderId = 1;

    public static final int backLeftDriveId = 14;
    public static final int backLeftSteerId = 9;
    public static final int backLeftEncoderId = 2;

    public static final int backRightDriveId = 2;
    public static final int backRightSteerId = 16;
    public static final int backRightEncoderId = 4;
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

    public static final int dPadUp = 0;
    public static final int dPadRight = 1;
    public static final int dPadDown = 2;
    public static final int dPadLeft = 3;

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

    public static final int kP = 0; // constant for Proportional
    public static final int kI = 0; // constant for Integral
    public static final int kD = 0; // constant for Derivative

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
     * This is the angular threshold to determine at what
     * point the maximum extenstion of the arm should be
     * limited by the arm hitting the ground.
     */
    public static final double kMaxGroundArmLengthThreshold = 0;
  }
}