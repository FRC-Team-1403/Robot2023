 
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
  }

  /*
   * Move the arm to an angle between 0 and 360 degrees where 0 is positive x axis and 90 is positive y axis
   * @param angle you want the arm move to 
   * @return the speed returned by the PID controller.
   */
  public double setArmRotation(double angle) {
    return m_pidArmRotation.calculate(getArmRotation(), angle);
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
    return m_telescopicMotor.getEmbeddedEncoder().getPositionTicks()
            * kArmLengthConversionFactor;
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
      if(isFrontSwitchActive() || isBackSwitchActive() || !isArmAngleWithinBounds() || getCurrentAmps() <= m_armConfig.kMaxAmperage) {
        setArmExtensionMotorSpeed(0);
        setArmAngleMotorSpeed(0);
        setWristMotorSpeed(0);
      }
  }
}