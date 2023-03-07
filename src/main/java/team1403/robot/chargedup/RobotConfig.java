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

  //Variables to used by all subsystems.
  public static final Dimension robotDimensions = new Dimension(0, 0, 0);
  public static final Dimension wristDimensions = new Dimension(0, 0, 0); //TODO


  public static double kRobotHeight = 32;
  public static double kHeightFromGround = 33.465;
  public static double kGroundToTopOfFrame = 1.465;
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

    public static final double kTrackWidth = Units.inchesToMeters(17.5);
    public static final double kWheelBase = Units.inchesToMeters(17.5);

    public static final SwerveDriveKinematics kDriveKinematics = 
        new SwerveDriveKinematics(
            // Front left
            new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
            // Front right
            new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
            // Back left
            new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
            // Back right
            new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

    public static final double frontLeftEncoderOffset = -(4.657165672020807 + Math.PI);
    public static final double frontRightEncoderOffset = -(3.127786826498822 + (2 * Math.PI));
    public static final double backLeftEncoderOffset = -(4.680175383839091 + Math.PI);
    public static final double backRightEncoderOffset = -(2.686000359587758);

    public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double kSteerReduction = (15.0 / 32.0) * (10.0 / 60.0);

    public static final double kSteerRelativeEncoderPositionConversionFactor = 2.0 * Math.PI
        * SwerveConfig.kSteerReduction;

    public static final double kSteerRelativeEncoderVelocityConversionFactor = 2.0 * Math.PI
        * SwerveConfig.kSteerReduction / 60.0;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kMaxSpeed = 14.5;

    public static final double kMaxAngularSpeed = 50;
    // (kMaxSpeed / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0)); // 39.795095397

    public static final double kVoltageSaturation = 12.0;
    public static final double kCurrentLimit = 40.0;

    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints
        = new TrapezoidProfile.Constraints(
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

    //Arm Ports
    public static final int wheelIntakeMotor = 5;
    public static final int telescopicArmMotor = 4;
    public static final int leftPivotMotor = 2;
    public static final int rightPivotMotor = 3;
    public static final int wristMotor = 1;

    // Swerve CanBus ids
    public static final int frontLeftDriveId = 6;
    public static final int frontLeftSteerId = 7;
    public static final int frontLeftEncoderId = 3;

    public static final int frontRightDriveId = 8;
    public static final int frontRightSteerId = 9;
    public static final int frontRightEncoderId = 1;

    public static final int backLeftDriveId = 10;
    public static final int backLeftSteerId = 11;
    public static final int backLeftEncoderId = 2;

    public static final int backRightDriveId = 12;
    public static final int backRightSteerId = 13;
    public static final int backRightEncoderId = 4;
  }

  /**
   * Ports on the RoboRIO.
   */
  public class RioPorts {

    
    public static final int kWristAbsoluteEncoder = 1; //DIO

    public static final int kArmAbsoluteEncoder = 3; ///Analog

    public static final int kArmLimitSwitch = 0; //DIO

    public static final int kExtensionMinMagneticSwitch = 2; //DIO
    public static final int kExtensionMaxMagneticSwitch = 3; //DIO
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
    public static final int pilotPort = 0;

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

    //Pivot
    public static final int kPArmPivot = 1;
    public static final int kIArmPivot = 0;
    public static final int kDArmPivot = 0;
    public static final double m_absolutePivotOffset = 64.4245336;
    public static final double kMaxPivotAngle = 250.273;
    public static final double kFrameAngle = 229.6427020308589;
    public static final double kMinPivotAngle = 132.211;
    public static final double kPivotAngleMaxAmperage = 40;
    public static final double kHorizonAngle = 209.099;

    //Wrist
    public static final double kPWristMotor = 0.95;
    public static final double kIWristMotor = 0;
    public static final double kDWristMotor = 90;
    public static final double kMaxWristAngle = 265;
    public static final double kMinWristAngle = 55.61740540442264; 
    public static final double kWristConversionFactor = 90.0 / 100;

    //Extension
    public static final double kPArmExtension = 0.3;
    public static final double kIArmExtension = 0;
    public static final double kDArmExtension = 0; 
    public static final double kMinArmExtension = 0; 
    public static final double kMaxArmExtension = 23.128;  //inches
    public static final double kExtensionConversionFactor = 1.0 / 6;  
    public static final double kArmExtensionMaxAmperage = 20; 
    public static final double maxVerticalAngle = Math.acos(Math.toRadians(44.3 / 60.218)); //TODO
    public static final double angleHittingRobot = 66; //TODO
    public static final double angleHittingGround = 80; //TODO
    public static final double kExtensionOffset = 8;

    //Dimensions
    public static final double kBaseArmLength = 31; //37 //28 inches
    public static final double kPhysicalArmMaxExtension = kBaseArmLength + kMaxArmExtension; //inches

    public static final double kArmWeight = 16; //Pounds
  }
}