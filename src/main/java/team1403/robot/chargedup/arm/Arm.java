 
package team1403.robot.chargedup.arm;

import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.AdvancedMotorController;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig;

/**
 * Arm subsytem with motors and PID control.
 * Order of operations for Arm Subsystem Angles
 * Arm angle, arm extension, and then wrist angle
 */
public class Arm extends CougarSubsystem {
  private final CougarSparkMax m_wheelIntakeMotor;
  private final CougarSparkMax m_telescopicMotor;
  private final CougarSparkMax m_leftAngledMotor;
  private final CougarSparkMax m_rightArmAngleMotor;
  private final CougarSparkMax m_wristAngleMotor;
  private final WpiLimitSwitch m_frontLimitSwitch;
  private final WpiLimitSwitch m_backLimitSwitch;
  private final PIDController m_pidArmAngle;
  private final PIDController m_pidWristAngle;
  private final PIDController m_pidArmLength;
  private DutyCycleEncoder m_absoluteEncoder;
  private double m_wheelSpeed;
  private double m_desiredArmAngle;
  private double m_desiredWristAngle;
  private double m_desiredArmExtension;
  
  // TODO find conversion factor for encoders

  /** 
   * Constructor arm defines variables for motors, as well as methods for
   * motor speed, and PID controls.
   */

  public Arm(CougarLibInjectedParameters injectedParameters) {
    super("TelescopicArm", injectedParameters);

    CougarLogger logger = getLogger(); //after game, to see what robot did

    m_wheelIntakeMotor = CougarSparkMax.makeBrushless("wheelIntake",
    RobotConfig.CanBus.wheelIntakeMotor, Type.kHallSensor, getLogger());
       
    m_telescopicMotor = CougarSparkMax.makeBrushless("telescopic",
    RobotConfig.CanBus.telescopicArmMotor, Type.kHallSensor, getLogger());

    m_leftAngledMotor = CougarSparkMax.makeBrushless("leftAngledArmMotor",
        RobotConfig.CanBus.leftAngledArmMotor, null, logger);

    m_rightArmAngleMotor = CougarSparkMax.makeBrushless("rightAngledArmMotor",
        RobotConfig.CanBus.rightAngledArmMotor, null, logger);

    m_wristAngleMotor = CougarSparkMax.makeBrushless("wristMotor",
    RobotConfig.CanBus.wristMotor, null, logger);

    m_frontLimitSwitch = new WpiLimitSwitch("frontSwitch",
    RobotConfig.RioPorts.exampleRailForwardLimitSwitch);

    m_backLimitSwitch = new WpiLimitSwitch("backSwitch",
    RobotConfig.RioPorts.exampleRailReverseLimitSwitch);

    m_rightArmAngleMotor.setInverted(true);

    m_rightArmAngleMotor.follow((AdvancedMotorController) m_leftAngledMotor);

    m_pidArmAngle = new PIDController(0, 0, 0);
    m_pidWristAngle = new PIDController(0, 0, 0);
    m_pidArmLength = new PIDController(0, 0, 0);

    m_desiredArmAngle = getArmAngle();
    m_desiredWristAngle = getWristAngle();
    m_desiredArmExtension = getArmExtension();
  }

  /**
   * Getter method for absolute encoder.
   *
   * @return absolute encoder
   */
  public DutyCycleEncoder getDutyCycleEncoder() {
    return m_absoluteEncoder; 
  }

  /**
   * Getter method for relative encoder,
   * sets the absolute encoder to relative encoder.
   */
  public void getRelativeEncoder() {   
    m_leftAngledMotor.getEmbeddedEncoder().setPositionOffset(
        m_absoluteEncoder.getAbsolutePosition());
  }

  /**
   * This method is the maximum the wrist can veritcally rotate.
   * will return the absolute wrist angle from current angle
   *
   * @param desiredWristAngle desired wrist angle
   * @param desiredArmAngle desired arm angle
   * @param currentWristAngle current wrist angle
   * @return absolute wrist angle
   */
  private double absoluteWristAngle(double desiredWristAngle, double desiredArmAngle,
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
    return Math.sin(relativeWristAngle - 180) * RobotConfig.Arm.wristDimensions.getLength();
  }

  /**
   * Calculates the theoretical arm length of the arm.
   *
   * @param absoluteArmAngle arm angle relative to ground
   * @return the theoretical arm length
   */
  private double theoreticalArmLength(double absoluteArmAngle) {
    return RobotConfig.Arm.robotDimensions.getLength() / Math.cos(270 - absoluteArmAngle);
  }
  
  /**
   * This method calculates the maximum arm
   * length the arm can go to without damaging the robot.
   *
   * @pararm absoluteArmAngle arm angle relative to ground
   * @param relativeWristAngle wrist angle relative to itself
   * @return the max arm length without hitting ground
   */
  private double maxGroundArmLength(double absoluteArmAngle, double relativeWristAngle) {
    return theoreticalArmLength(absoluteArmAngle) - wristVerticleOccupation(relativeWristAngle)
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
    if (angle > RobotConfig.Arm.kMaxArmRotation) {
      angle = RobotConfig.Arm.kMaxArmRotation;
    } else  if (angle < RobotConfig.Arm.kMinArmRotation) {
      angle = RobotConfig.Arm.kMinArmRotation;
    }

    return angle;
  }

  /**
   * Helper function for m_desiredArmExtension.
   * checking the desired arm angle against the max ground arm length,
   * this would make sure the arm doesn't hit the ground, also checks the 
   * length to make sure it doesn't hit the ground
   *
   * @param length current length of arm
   * @param desiredArmAngle desired angle of arm
   * @param desiredRelativeWristAngle desired wrist angle, relative to itself
   * @return current length of arm
   */
  private double limitArmExtension(double length, double desiredArmAngle,
      double desiredRelativeWristAngle) {
    desiredArmAngle = 270 - desiredArmAngle;
    double max = 0;

    /*
     * Limit for the desired arm angle, if over the limit, then
     * sets max equal to the limit, otherwise 
     * sets the max = to whatever the desired arm angle is
     */
    if (desiredArmAngle < RobotConfig.Arm.kMaxGroundArmLengthThreshold) {
      max = maxGroundArmLength(desiredArmAngle, desiredRelativeWristAngle);
    } else {
      max = RobotConfig.Arm.kMaxArmExtension;
    }

    //Checks length against max.
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
    if (angle > RobotConfig.Arm.kMaxWristRotation) {
      angle = RobotConfig.Arm.kMaxWristRotation;
    } else  if (angle < RobotConfig.Arm.kMinWristRotation) {
      angle = RobotConfig.Arm.kMinWristRotation;
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
    m_desiredArmExtension = limitArmExtension(armExtension, m_desiredArmAngle, wristAngle);
    m_desiredWristAngle = absoluteWristAngle(wristAngle, m_desiredArmAngle, getWristAngle());
    m_desiredWristAngle = limitWristAngle(wristAngle);
    m_wheelSpeed = speed;
  }

  /**
   * sets the intake motor's speed.
   *
   * @param speed speed of motor
   */
  public void setWheelSpeed(double speed) {
    m_wheelIntakeMotor.setSpeed(speed);
  }

  /**
   * Move the arm to the desired angle where 0 is positive x axis
   * and 90 is positive y axis.
   *
   * @param angle you want the arm move to
   */
  public void setArmAngle(double angle) {
    m_leftAngledMotor.setSpeed(m_pidArmAngle.calculate(getArmAngle(), angle));
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
    return m_leftAngledMotor.getEmbeddedEncoder().getPositionValue()
      * RobotConfig.Arm.kArmConversionFactor;
  }

  /**
   * Getter for wrist angle motor position.
   *
   * @return position of wrist angle motor
   */
  public double getWristAngle() {
    return m_wristAngleMotor.getEmbeddedEncoder().getPositionValue()
      * RobotConfig.Arm.kWristConversionFactor;
  }

  /**
   * Getter for arm extension motor position.
   *
   * @return position of arm extension motor
   */
  public double getArmExtension() {
    return (m_telescopicMotor.getEmbeddedEncoder().getPositionValue()
            * RobotConfig.Arm.kArmLengthConversionFactor) * RobotConfig.Arm.kAngleToMeters;
  }

  /**
   * Setter for wheel intake motor speed.
   *
   * @param speed speed for motor
   */
  public void setWheelIntakeMotorSpeed(double speed) {
    m_wheelIntakeMotor.setSpeed(speed);
  }

  /**
   * Setter for arm extension motor speed.
   *
   * @param speed speed for motor
   */
  public void setArmExtensionMotorSpeed(double speed) {
    m_telescopicMotor.setSpeed(speed);
  }
     
  /**
   * Setter for arm angle motor speed.
   *
   * @param speed speed for motor
   */
  public void setArmAngleMotorSpeed(double speed) {
    m_leftAngledMotor.setSpeed(speed);
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
    m_telescopicMotor.setSpeed(0);
    m_leftAngledMotor.setSpeed(0);
    m_wristAngleMotor.setSpeed(0);
    m_wheelIntakeMotor.setSpeed(0);
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
  public boolean isBackSwitchActive() {
    return m_backLimitSwitch.isTriggered();
  }

  /**
   * Getter for current amps.
   *
   * @return current amps
   */
  public double getCurrentAmps() {
    return m_leftAngledMotor.getEmbeddedCurrentSensor().getAmps();
  }

  /**
   * Getter for limiting the arm angle.
   *
   * @return limit for arm angle
   */
  private boolean isArmAngleWithinBounds() {
    return getArmAngle() <= RobotConfig.Arm.kMaxArmRotation && getArmAngle()
      >= RobotConfig.Arm.kMinArmRotation;
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
    return getWristAngle() <= RobotConfig.Arm.kMaxWristRotation && getWristAngle()
      >= RobotConfig.Arm.kMinWristRotation;
  }

  @Override
  public void periodic() {
    if (isFrontSwitchActive() || isBackSwitchActive() || !isArmAngleWithinBounds()
        || getCurrentAmps() <= RobotConfig.Arm.kMaxAmperage) {
      setArmAngleMotorSpeed(0);
      return;
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
    setWheelSpeed(m_wheelSpeed);
  }
}
