 
package team1403.robot.chargedup.armSubsystem;

import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.AdvancedMotorController;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig;
import team1403.robot.chargedup.RobotConfig.CanBus;
import team1403.robot.chargedup.RobotConfig.RioPorts;

/**
 * Arm subsytem with motors and PID control.
 * Order of operations for Arm Subsystem Angles
 * Arm angle, arm extension, and then wrist angle
 */
public class Arm extends CougarSubsystem {
  private final RobotConfig.Arm m_armConfig;
  private final CougarSparkMax m_telescopicMotor;
  private final CougarSparkMax m_leftAngledMotor;
  private final CougarSparkMax m_rightArmAngleMotor;
  private final CougarSparkMax m_wristAngleMotor;
  private final WpiLimitSwitch m_frontLimitSwitch;
  private final WpiLimitSwitch m_backLimitSwitch;
  private final PIDController m_pidArmRotation;
  private final PIDController m_pidWristRotation;
  private final PIDController m_pidArmLength;
  public double m_desiredArmAngle;
  public double m_desiredWristAngle;
  public double m_desiredArmExtension;
  
  

  // TODO find conversion factor
  private static final double kArmConversionFactor = 1;
  private static final double kWristConversionFactor = 2;
  private static final double kArmLengthConversionFactor = 3;

  /** 
   * Constructor arm defines variables for motors, as well as methods for
   * motor speed, and PID controls.
   */
  public Arm(CougarLibInjectedParameters injectedParameters, RobotConfig robotConfig) {
    super("TelescopicArm", injectedParameters);
    this.m_armConfig = robotConfig.arm;

    CanBus can = robotConfig.canBus; //This is specfic to SparkCanMax motor
    RioPorts ports = robotConfig.ports;

    CougarLogger logger = getLogger(); //after game, to see what robot did
       
    m_telescopicMotor = CougarSparkMax.makeBrushless("telescopic", can.telescopicArmMotor, 
                           Type.kHallSensor, getLogger());

    m_leftAngledMotor = CougarSparkMax.makeBrushless("leftAngledArmMotor",
        can.leftAngledArmMotor, 
                           null, logger);

    m_rightArmAngleMotor = CougarSparkMax.makeBrushless("rightAngledArmMotor",
        can.rightAngledArmMotor, null, logger);

    m_wristAngleMotor = CougarSparkMax.makeBrushless("wristMotor", can.wristMotor, null, logger);

    m_frontLimitSwitch = new WpiLimitSwitch("frontrSwitch", ports.exampleRailForwardLimitSwitch);

    m_backLimitSwitch = new WpiLimitSwitch("backSwitch", ports.exampleRailReverseLimitSwitch);

    m_rightArmAngleMotor.setInverted(true);

    m_rightArmAngleMotor.follow((AdvancedMotorController) m_leftAngledMotor);

    m_pidArmRotation = new PIDController(0, 0, 0);
    m_pidWristRotation = new PIDController(0, 0, 0);
    m_pidArmLength = new PIDController(0, 0, 0);

    m_desiredArmAngle = getArmRotation();
    m_desiredWristAngle = getWristRotation();
    m_desiredArmExtension = getArmExtension();
  }

  /**
   * This method is the maximum the wrist can veritcally rotate. It can use currentWristAngle,
   * because it is only converting from 
   * 
   * @param armAngle
   * @return
   */
  private double absoluteWristAngle(double desiredWristAngle, double desiredArmAngle, double currentWristAngle) {
    return currentWristAngle - (desiredArmAngle - 180) - 180 + desiredWristAngle;
  }  
  
  /*
   * Calculates the space the wrist takes up vertically
   */
  private double wristVerticleOccupation(double relativeWristAngle) {
    return Math.sin(relativeWristAngle - 180) * m_armConfig.wristDimensions.getLength();
  }

  /*
   * Calculates the theoretical arm length of the arm.
   */
  private double theoreticalArmLength(double absoluteArmAngle) {
    return m_armConfig.robotDimensions.getLength() / Math.cos(270 - absoluteArmAngle);
  }
  
  /*
   * This method calculates the maximum arm length the arm can go to without damaging the robot
   */
  public double maxGroundArmLength(double absoluteArmAngle, double relativeWristAngle) {
    return theoreticalArmLength(absoluteArmAngle) - wristVerticleOccupation(relativeWristAngle) - m_armConfig.kMaxArmLengthOffset;
  }

  /**
   * Helper function for m_desiredArmAngle
   * 
   * @param angle
   * @return
   * 
   */

  public double normalizeArmAngle(double angle) {
    if(angle > m_armConfig.kMaxArmRotation) {
      angle = m_armConfig.kMaxArmRotation;
    } else  if(angle < m_armConfig.kMinArmRotation) {
      angle = m_armConfig.kMinArmRotation;
    }

    return angle;
  }
  /**
   * Helper function for m_desiredArmExtension
   * 
   * @param angle
   * @return
   */
  public double normalizeArmExtension(double length, double desiredArmAngle, double desiredRelativeWristAngle) {
    desiredArmAngle = 270 - desiredArmAngle;
    double max = 0;

    if(desiredArmAngle < m_armConfig.kMaxGroundArmLengthThreshold) {
      max = maxGroundArmLength(desiredArmAngle, desiredRelativeWristAngle);
    } else {
      max = m_armConfig.kMaxArmExtension;
    }


    if(length > max) {
      length = max;
    } else if (length < 0) {
      length = 0;
    }

    return length;
  }

public double normalizeWristAngle(double angle) {
    if(angle > m_armConfig.kMaxWristRotation) {
      angle = m_armConfig.kMaxWristRotation;
    } else  if(angle < m_armConfig.kMinWristRotation) {
      angle = m_armConfig.kMinWristRotation;
    }

    return angle;
  }

  /*
   * This method is being called inside Periodic, and sets up the variables to be changed to the setpoints.
   */
  public void moveArm(double armAngle, double armExtension, double wristAngle) {
    m_desiredArmAngle = normalizeArmAngle(armAngle);
    m_desiredArmExtension = normalizeArmExtension(armExtension, m_desiredArmAngle, wristAngle);
    m_desiredWristAngle = absoluteWristAngle(wristAngle, m_desiredArmAngle, getWristRotation());
    m_desiredWristAngle = normalizeWristAngle(wristAngle);
  }


  /*
   * Move the arm to an angle between 0 and 360 degrees where 0 is positive x axis and 90 is positive y axis
   * @param angle you want the arm move to 
   * @return the speed returned by the PID controller.
   */
  public void setArmRotation(double angle) {
    m_leftAngledMotor.setSpeed(m_pidArmRotation.calculate(getArmRotation(), angle));
  }

  /*
   * Method to set wrist rotation, PID calculated from encoder measurements.
   */
  public double setWristRotation(double angle) {
    return m_pidWristRotation.calculate(getWristRotation(), angle);
  }
  
  public double setArmExtension(double angle) {
    return m_pidArmLength.calculate(getArmExtension(), angle);
  }

  //Getters for motor positions

  public double getArmRotation() {
    return m_leftAngledMotor.getEmbeddedEncoder().getPositionTicks() * kArmConversionFactor;
  }

  public double getWristRotation() {
    return m_wristAngleMotor.getEmbeddedEncoder().getPositionTicks() * kWristConversionFactor;
  }

  public double getArmExtension() {
    return (m_telescopicMotor.getEmbeddedEncoder().getPositionTicks()
            * kArmLengthConversionFactor) * m_armConfig.kAngleToMeters;
  }

  //Setters for motor speeds
     
  public void setArmExtensionMotorSpeed(double speed) {
    m_telescopicMotor.setSpeed(speed);
  }
     
  public void setArmAngleMotorSpeed(double speed) {
    m_leftAngledMotor.setSpeed(speed);
  }

  public void setWristMotorSpeed(double speed) {
    m_wristAngleMotor.setSpeed(speed);
  }

  public void setStopArm(double speed) {
    m_telescopicMotor.setSpeed(0);
    m_leftAngledMotor.setSpeed(0);
    m_wristAngleMotor.setSpeed(0);
  }
     
  //Getters

  public boolean isFrontSwitchActive() {
    return m_frontLimitSwitch.isTriggered();
  }

  public boolean isBackSwitchActive() {
    return m_backLimitSwitch.isTriggered();
  }

  public RobotConfig.Arm getArmConfig() {
    return m_armConfig;
  }
     
  public double getCurrentAmps() {
    return m_leftAngledMotor.getEmbeddedCurrentSensor().getAmps();
  }

  private boolean isArmAngleWithinBounds() {
    return getArmRotation() <= m_armConfig.kMaxArmRotation && getArmRotation() >= m_armConfig.kMinArmRotation;
  }

  @Override
  public void periodic() {
    
    
    setArmRotation(m_desiredArmAngle);
    setWristRotation(m_desiredWristAngle);
    setArmExtension(m_desiredArmExtension);


    //TODO, add constraints for angles

    if(isFrontSwitchActive() || isBackSwitchActive() || !isArmAngleWithinBounds() || getCurrentAmps() <= m_armConfig.kMaxAmperage) {
      setArmExtensionMotorSpeed(0);
      setArmAngleMotorSpeed(0);
      setWristMotorSpeed(0);
    }
  }
}