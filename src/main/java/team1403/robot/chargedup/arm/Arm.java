
package team1403.robot.chargedup.arm;

import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.AdvancedMotorController;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiAnalogDevice;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig;

/**
 * Arm subsytem with motors and PID control.
 * Order of operations for Arm Subsystem Angles
 * Arm angle, arm extension, and then wrist angle
 */
public class Arm extends CougarSubsystem {
  // private final CougarSparkMax m_wheelIntakeMotor;
  private final CougarSparkMax m_extensionMotor;
  private final CougarSparkMax m_leftArmAngleMotor;
  private final CougarSparkMax m_rightArmAngleMotor;
  private final CougarSparkMax m_wristAngleMotor;
  private final WpiLimitSwitch m_frontLimitSwitch;
  private final WpiLimitSwitch m_telescopicLimitSwitch;
  private final PIDController m_pidArmAngle;
  private final PIDController m_pidWristAngle;
  private final PIDController m_pidArmLength;
  private WpiAnalogDevice m_absoluteArmEncoder;
  private Encoder m_absoluteWristEncoder;
  private Double m_wheelSpeed;
  private Double m_desiredArmAngle;
  private Double m_desiredWristAngle;
  private Double m_desiredArmExtension;
  
  /** 
   * Constructor arm defines variables for motors, as well as methods for
   * motor speed, and PID controls.
   */

  public Arm(CougarLibInjectedParameters injectedParameters) {
    super("TelescopicArm", injectedParameters);

    CougarLogger logger = getLogger(); //after game, to see what robot did

    // m_wheelIntakeMotor = CougarSparkMax.makeBrushless("wheelIntake",
    // RobotConfig.CanBus.wheelIntakeMotor, Type.kHallSensor, getLogger());
       
    m_extensionMotor = CougarSparkMax.makeBrushless("telescopic",
    RobotConfig.CanBus.telescopicArmMotor, Type.kHallSensor, getLogger());

    m_leftArmAngleMotor = CougarSparkMax.makeBrushless("leftAngledArmMotor",
        RobotConfig.CanBus.leftPivotMotor, Type.kHallSensor, logger);

    m_rightArmAngleMotor = CougarSparkMax.makeBrushless("rightAngledArmMotor",
        RobotConfig.CanBus.rightPivotMotor, Type.kHallSensor, logger);

    m_wristAngleMotor = CougarSparkMax.makeBrushless("wristMotor",
    RobotConfig.CanBus.wristMotor, Type.kHallSensor, logger);

    m_frontLimitSwitch = new WpiLimitSwitch("frontSwitch",
    RobotConfig.RioPorts.exampleRailForwardLimitSwitch);

    m_telescopicLimitSwitch = new WpiLimitSwitch("telescopicLimitSwitch",
    RobotConfig.RioPorts.exampleRailReverseLimitSwitch);

    m_rightArmAngleMotor.setInverted(true);

    m_rightArmAngleMotor.follow((AdvancedMotorController) m_leftArmAngleMotor);

    m_extensionMotor.getEmbeddedEncoder().setPositionConversionFactor(
        RobotConfig.Arm.kArmLengthConversionFactor);

    m_wristAngleMotor.getEmbeddedEncoder().setPositionConversionFactor(
        RobotConfig.Arm.kWristConversionFactor);

    m_pidArmAngle = new PIDController(0, 0, 0);
    m_pidWristAngle = new PIDController(0, 0, 0);
    m_pidArmLength = new PIDController(0, 0, 0);

    m_leftArmAngleMotor.setPidGains(RobotConfig.Arm.kPArmPivot, RobotConfig.Arm.kIArmPivot,
        RobotConfig.Arm.kDArmPivot);
      
    m_extensionMotor.setPidGains(RobotConfig.Arm.kPArmExtension, RobotConfig.Arm.kIArmExtension,
        RobotConfig.Arm.kDArmExtension);

    m_wristAngleMotor.setPidGains(RobotConfig.Arm.kPWristMotor, RobotConfig.Arm.kIWristMotor,
        RobotConfig.Arm.kDWristMotor);

    m_desiredArmAngle = getArmAngle();
    m_desiredWristAngle = getWristAngle();
    m_desiredArmExtension = getArmExtension();

    m_wristAngleMotor.getEmbeddedEncoder().setPositionOffset(180);
    resetArmAngleEncoder();
  }

  /**
   * Getter method for absolute arm encoder.
   *
   * @return arm absolute encoder
   */
  public WpiAnalogDevice getArmEncoder() {
    return m_absoluteArmEncoder; 
  }

  /**
   * Getter method for absolute wrist encoder.
   * 
   * @return wrist absolute encoder.
   */
  public Encoder getWristEncoder() {
    return m_absoluteWristEncoder;
  }

  public void resetArmAngleEncoder() {
    m_leftArmAngleMotor.getEmbeddedEncoder().setPositionOffset(getArmAngle() - 180);
  }

  /**
   * Getter method for relative arm encoder,
   * sets the absolute wrist encoder to relative arm encoder.
   */
  public void getRelativeArmEncoder() {
    m_leftArmAngleMotor.getEmbeddedEncoder().setPositionOffset(
        m_absoluteArmEncoder.getAnalogValue());
  }

  /**
   * Getter method for relative wrist encoder,
   * sets the absolute wrist encoder to relative wrist encoder.
   */
  public void getRelativeWristEncoder() {
    m_wristAngleMotor.getEmbeddedEncoder().setPositionOffset(
        m_absoluteWristEncoder.get());
  }

  /**
   * Will return the absolute wrist angle from current angle.
   *
   * @param desiredWristAngle desired wrist angle.
   * @param desiredArmAngle desired arm angle.
   * @param currentWristAngle current wrist angle.
   * @return absolute wrist angle.
   */
  public double absoluteWristAngle(double desiredWristAngle, double desiredArmAngle,
      double currentWristAngle) {
    return currentWristAngle - (desiredArmAngle - 180) - 180 + desiredWristAngle;
  }
  
  /**
   * Calculates the space the wrist takes up vertically.
   *
   * @param relativeWristAngle wrist angle relative to itself
   * @return space wrist takes up vertically
   */
  private double wristVerticleOccupation(double relativeWristAngle) {
    return Math.sin(relativeWristAngle - 180)
       * RobotConfig.Arm.wristDimensions.getHeight(RobotConfig.Arm.kPhysicalArmMaxExtension);
  }

  /**
   * Calculates the theoretical max arm length of the arm given the arm angle.
   *
   * @param absoluteArmAngle arm angle relative to ground
   * @return the theoretical arm length
   */
  public double theoreticalArmLength(double absoluteArmAngle, double height) {
    return height / Math.cos(270 - absoluteArmAngle);
  }
  
  /**
   * This method calculates the maximum arm
   * length the arm can go to without damaging the robot.
   *
   * @pararm absoluteArmAngle arm angle relative to ground
   * @param relativeWristAngle wrist angle relative to itself
   * @return the max arm length without hitting ground
   */
  public double maxGroundArmLength(double absoluteArmAngle,
      double relativeWristAngle, double height) {
    return theoreticalArmLength(absoluteArmAngle, height)
      - wristVerticleOccupation(relativeWristAngle)
      - RobotConfig.Arm.kMaxArmLengthOffset;
  }

  /**
   * Helper function for m_desiredArmAngle.
   *
   * @param angle current angle of arm
   * @return current angle of arm
   * 
   */
  private double limitArmAngle(double angle) {

    if (angle > RobotConfig.Arm.kMaxPivotAngle) {
      angle = RobotConfig.Arm.kMaxPivotAngle;
    } else  if (angle < RobotConfig.Arm.kMinPivotAngle) {
      angle = RobotConfig.Arm.kMinPivotAngle;
    }

    if (angle > RobotConfig.Arm.maxVerticalAngle) {
      angle = RobotConfig.Arm.maxVerticalAngle;
    }
    
    return angle;
  }

  /**
   * Helper function for m_desiredArmExtension.
   * checking the desired arm angle against the max ground arm length,
   * this would make sure the arm doesn't hit the ground, also checks the 
   * length to make sure it doesn't hit the ground
   *
   * @param angle current angle of arm
   * @param length current length of arm
   * @return current length of arm
   */
  public double limitArmExtension(double angle, double length) {
    double max = 0;

    if (angle > 0 && angle < RobotConfig.Arm.angleHittingRobot) {
      max = maxGroundArmLength(getArmAngle(), getWristAngle(), 
          RobotConfig.kRobotHeight - RobotConfig.kFrameHeight);

    } else if (angle > RobotConfig.Arm.angleHittingRobot && angle 
          < RobotConfig.Arm.angleHittingGround) {
      max = maxGroundArmLength(getArmAngle(), getWristAngle(), RobotConfig.kHeightFromGround);
    } else {
      max = RobotConfig.Arm.kPhysicalArmMaxExtension;
    }

    if (length > max) {
      length = max;
    } else if (length < 0) {
      length = 0;
    }
    
    return length;
  }

  /**
   * Checks current angle against max angle, if it is over the max,
   * then sets it eequal to the max.
   * Checks current angle against min angle, if it is over the min,
   * then sets it eequal to the min.
   *
   * @param angle current angle
   * 
   * @return angle
   */
  private double limitWristAngle(double angle) {
    if (angle > RobotConfig.Arm.kMaxWristAngle) {
      angle = RobotConfig.Arm.kMaxWristAngle;
    } else  if (angle < RobotConfig.Arm.kMinWristAngle) {
      angle = RobotConfig.Arm.kMinWristAngle;
    }

    return angle;
  }

  /**
   * This method is being called inside the periodic method, and sets up the
   * desired angles and extentions to be changed to the setpoints.
   *
   * @param armAngle current arm angle
   * @param armExtension current arm extension
   * @param wristAngle current wrist angle
   * @param speed current wheel speed, between -1 and 1 inclusive
   */
  public void moveArm(double armAngle, double armExtension, double wristAngle, double speed) {
    m_desiredArmAngle = limitArmAngle(armAngle);
    m_desiredArmExtension = limitArmExtension(getArmAngle(), getArmExtension());
    m_desiredWristAngle = absoluteWristAngle(wristAngle, m_desiredArmAngle, getWristAngle());
    m_desiredWristAngle = limitWristAngle(wristAngle);
  }

  // /**
  //  * sets the intake motor's speed.
  //  *
  //  * @param speed speed of motor
  //  */
  // public void setWheelSpeed(double speed) {
  //   m_wheelIntakeMotor.setSpeed(speed);
  // }

  /**
   * Move the arm to the desired angle where 0 is positive x axis
   * and 90 is positive y axis.
   *
   * @param angle you want the arm move to
   */
  public void setArmAngle(double angle) {
    m_leftArmAngleMotor.setSpeed(m_pidArmAngle.calculate(getArmAngle(), angle));
  }

  /**
   * Move the wrist to the desired angle where 0 is positive x axis
   * and 90 is positive y axis.
   *
   * @param angle you want the wrist move to 
   * @return the speed returned by the PID controller.
   */
  public double setWristAngle(double angle) {
    return m_pidWristAngle.calculate(getWristAngle(), angle);
  }
  
  /**
   * Move the arm to a length between 0 and max
    length arm can go to where 0 is positive x axis
   * and 90 is positive y axis.
   *
   * @param length you want the arm move to
   * @return the speed returned by the PID controller.
   */
  public double setArmExtension(double length) {
    return m_pidArmLength.calculate(getArmExtension(), length);
  }
  
  /**
   * Getter for arm angle motor position.
   *
   * @return position of arm angle motor
   */
  public double getArmAngle() {
    return m_leftArmAngleMotor.getEmbeddedEncoder().getPositionValue();
  }

  /**
   * Getter for wrist angle motor position.
   *
   * @return position of wrist angle motor
   */
  public double getWristAngle() {
    return m_wristAngleMotor.getEmbeddedEncoder().getPositionValue();
  }

  /**
   * Getter for arm extension motor position.
   *
   * @return position of arm extension motor
   */
  public double getArmExtension() {
    return (m_extensionMotor.getEmbeddedEncoder().getPositionValue());
  }

  // /**
  //  * Setter for wheel intake motor speed.
  //  *
  //  * @param speed speed for motor
  //  */
  // public void setWheelIntakeMotorSpeed(double speed) {
  //   m_wheelIntakeMotor.setSpeed(speed);
  // }

  /**
   * Setter for arm extension motor speed.
   *
   * @param speed speed for motor
   */
  public void setArmExtensionMotorSpeed(double speed) {
    m_extensionMotor.setSpeed(speed);
  }
     
  /**
   * Setter for arm angle motor speed.
   *
   * @param speed speed for motor
   */
  public void setArmAngleMotorSpeed(double speed) {
    m_leftArmAngleMotor.setSpeed(speed);
  }

  /**
   * Setter for wrist angle motor speed.
   *
   * @param speed speed for motor
   */
  public void setWristMotorSpeed(double speed) {
    m_wristAngleMotor.setSpeed(speed);
  }

  /**
   * Stops all motors for arm, as well as intake.
   */
  public void stopArm() {
    m_extensionMotor.setSpeed(0);
    m_leftArmAngleMotor.setSpeed(0);
    m_wristAngleMotor.setSpeed(0);
    // m_wheelIntakeMotor.setSpeed(0);
  }
     
  /**
   * Getter for front limit switch.
   *
   * @return if front limit switch is triggered
   */
  public boolean isFrontSwitchActive() {
    return m_frontLimitSwitch.isTriggered();
  }

  /**
   * Gtter for back limit switch.
   *
   * @return if back limit switch is triggered
   */
  public boolean isTelescopicSwitchActive() {
    return m_telescopicLimitSwitch.isTriggered();
  }

  /**
   * Getter for current amps.
   *
   * @return current amps
   */
  public double getCurrentArmAngleAmps() {
    return m_leftArmAngleMotor.getEmbeddedCurrentSensor().getAmps();
  }

  public double getCurrentArmExtensionAmps() {
    return m_extensionMotor.getEmbeddedCurrentSensor().getAmps();
  }

  /**
   * Getter for limiting the arm angle.
   *
   * @return limit for arm angle
   */
  private boolean isArmAngleWithinBounds() {
    return getArmAngle() <= RobotConfig.Arm.kMaxPivotAngle && getArmAngle()
      >= RobotConfig.Arm.kMinPivotAngle;
  }

  /**
   * Getter for limiting the arm extension.
   * 
   * @return limit for arm extension
   */
  private boolean isArmExtensionWithinBounds() {
    return getArmExtension() <= RobotConfig.Arm.kMaxArmExtension && getArmExtension()
      >= RobotConfig.Arm.kMinArmExtension;
  }

  /**
   * Getter for limiting the wrist angle.
   * 
   * @return limit for wrist angle
   */
  private boolean isWristAngleWithinBounds() {
    return getWristAngle() <= RobotConfig.Arm.kMaxWristAngle && getWristAngle()
      >= RobotConfig.Arm.kMinWristAngle;
  }

  @Override
  public void periodic() {
    if (isFrontSwitchActive() || !isArmAngleWithinBounds()
        || getCurrentArmAngleAmps() <= RobotConfig.Arm.kArmAngleMaxAmperage
        || getCurrentArmExtensionAmps() <= RobotConfig.Arm.kArmExtensionMaxAmperage) {
      setArmAngleMotorSpeed(0);
      return;
    }

    if (isTelescopicSwitchActive()) {
      setArmExtensionMotorSpeed(0);
    }

    if (isWristAngleWithinBounds()) {
      setWristMotorSpeed(0);
      return;
    }

    if (isArmExtensionWithinBounds()) {
      setArmExtensionMotorSpeed(0);
      return;
    }

    setArmAngle(m_desiredArmAngle);
    setWristAngle(m_desiredWristAngle);
    setArmExtension(m_desiredArmExtension);
    // setWheelSpeed(m_wheelSpeed);
  }
}
