package team1403.robot.chargedup.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.lib.util.ArmState;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig.Arm;
import team1403.robot.chargedup.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class Arm_Subsystem extends CougarSubsystem {
  // Wrist
  private final CougarSparkMax m_wristMotor;
  private final DutyCycleEncoder m_wristAbsoluteEncoder;
  private double m_absoluteWristOffset = 30;

  // Arm
  private final CANSparkMax m_leftPivotMotor;
  private final CANSparkMax m_rightPivotMotor;
  private final AnalogEncoder m_armAbsoluteEncoder;
  private final PIDController m_pivotPid;
  private final WpiLimitSwitch m_maxArmLimitSwitch;

  // Intake
  private final CANSparkMax m_intakeMotor;

  // Telescope
  private final CANSparkMax m_extensionMotor;
  private final DigitalInput m_minMagneticSwitch;
  private final DigitalInput m_maxMagneticSwitch;
  private double m_extensionLimitSwitchOffset;

  // Setpoints
  private double m_wristAngle;
  private double m_pivotAngle;
  private double m_intakeSpeed;
  private double m_extensionLength;

  public Arm_Subsystem(CougarLibInjectedParameters injectedParameters) {
    super("Arm", injectedParameters);
    CougarLogger logger = getLogger();

    m_wristMotor = CougarSparkMax.makeBrushless("Wrist Motor", RobotConfig.CanBus.wristMotor, Type.kHallSensor, logger);
    m_wristAbsoluteEncoder = new DutyCycleEncoder(RobotConfig.RioPorts.kWristAbsoluteEncoder);

    m_leftPivotMotor = new CANSparkMax(RobotConfig.CanBus.leftPivotMotor, MotorType.kBrushless);
    m_rightPivotMotor = new CANSparkMax(RobotConfig.CanBus.rightPivotMotor, MotorType.kBrushless);
    m_armAbsoluteEncoder = new AnalogEncoder(RobotConfig.RioPorts.kArmAbsoluteEncoder);

    m_intakeMotor = new CANSparkMax(RobotConfig.CanBus.wheelIntakeMotor, MotorType.kBrushed);

    m_extensionMotor = new CANSparkMax(RobotConfig.CanBus.telescopicArmMotor, MotorType.kBrushless);

    m_maxArmLimitSwitch = new WpiLimitSwitch("maxArmLimitSwitch",
    RobotConfig.RioPorts.kArmLimitSwitch);

    configWristMotor();
    configEncoders();

    // TODO, tune feedforward with arm pivot
    // TODO Constraints
    m_pivotPid = new PIDController(0.025, RobotConfig.Arm.kIArmPivot, RobotConfig.Arm.kDArmPivot);

    m_minMagneticSwitch = new DigitalInput(RobotConfig.RioPorts.kExtensionMinMagneticSwitch);
    m_maxMagneticSwitch = new DigitalInput(RobotConfig.RioPorts.kExtensionMaxMagneticSwitch);

  }

  private void configEncoders() {
    // Wrist encoders
    m_wristMotor.getEncoder().setPositionConversionFactor(RobotConfig.Arm.kWristConversionFactor);
    new Thread(() -> {
      try {
        Thread.sleep(2000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      double wristAngle = getWristAbsoluteAngle();
      m_wristMotor.getEncoder().setPosition(wristAngle);
      m_wristAngle = wristAngle;
    }).start();

    //Telescopic encoders
    m_extensionMotor.getEncoder().setPositionConversionFactor(RobotConfig.Arm.kExtensionConversionFactor);

    // Arm encoders
    m_leftPivotMotor.getEncoder().setPositionConversionFactor(1.53285964552);
    m_leftPivotMotor.getEncoder().setPosition(getAbsolutePivotAngle());
  }

  private void configWristMotor() {
    // Wrist
    SparkMaxPIDController wristController = m_wristMotor.getPIDController();
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setInverted(false);
    m_wristMotor.enableVoltageCompensation(12);
    m_wristMotor.setSmartCurrentLimit(20);
    m_wristMotor.setRampRate(0.25);

    wristController.setP(Arm.kPWristMotor);
    wristController.setI(Arm.kIWristMotor);
    wristController.setD(Arm.kDWristMotor);
    wristController.setFeedbackDevice((MotorFeedbackSensor) m_wristMotor.getEncoder());
    wristController.setPositionPIDWrappingEnabled(false);

    // Pivot
    m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
    m_leftPivotMotor.enableVoltageCompensation(12);
    m_leftPivotMotor.setSmartCurrentLimit(25);
    m_rightPivotMotor.follow(m_leftPivotMotor, true);

    // Extension
    SparkMaxPIDController extensionController = m_extensionMotor.getPIDController();
    m_extensionMotor.setIdleMode(IdleMode.kBrake);
    m_extensionMotor.enableVoltageCompensation(12);
    m_extensionMotor.setSmartCurrentLimit(20);
    m_extensionMotor.setOpenLoopRampRate(0.25);

    extensionController.setP(RobotConfig.Arm.kPArmExtension);
    extensionController.setI(RobotConfig.Arm.kIArmExtension);
    extensionController.setD(RobotConfig.Arm.kDArmExtension);
    extensionController.setFeedbackDevice(m_extensionMotor.getEncoder());
    extensionController.setPositionPIDWrappingEnabled(false);
  }

  // Wrist Methods
  public double getWristAbsoluteAngle() {
    double value = (m_wristAbsoluteEncoder.getAbsolutePosition() * 360) + m_absoluteWristOffset;

    if (value < 0) {
      value += 360;
    }
    if (value > 360) {
      value -= 360;
    }
    return value;
  }

  private void setAbsoluteWristAngle(double absoluteWristAngle) {
    m_wristMotor.getPIDController().setReference(absoluteWristAngle, CANSparkMax.ControlType.kPosition);
  }

  public double absoluteWristAngle(double desiredWristAngle, double desiredArmAngle,
      double currentWristAngle) {
    return currentWristAngle - (desiredArmAngle - 180) - 180 + desiredWristAngle;
  }

  public double limitWristAngle(double angle) {
    return MathUtil.clamp(angle, Arm.kMinWristAngle, Arm.kMaxWristAngle);
  }

  /**
   * Checks if the given angle is in the bounds of the wrist.
   * 
   * @param angle the given angle
   * @return true if the given angle is in the bounds of the wrist.
   */
  private boolean isInWristBounds(double angle) {
    return (angle > Arm.kMinWristAngle && angle < Arm.kMaxWristAngle);
  }

  // Pivot Methods

  /**
   * Getter for arm limit switch.
   *
   * @return if arm limit switch is triggered
   */
  public boolean isArmSwitchActive() {
    return m_maxArmLimitSwitch.isTriggered();
  }

  public double getAbsolutePivotAngle() {
    double value = (m_armAbsoluteEncoder.getAbsolutePosition() * 360) + 38.1111122229;

    if (value < 0) {
      value += 360;
    }
    if (value > 360) {
      value -= 360;
    }
    return value;
  }

  private void setAbsolutePivotAngle(double desiredAngle) {
    //Feedforward
    double currentAngle = getAbsolutePivotAngle();
    double normalizedCurrentAngle = currentAngle;
    while(normalizedCurrentAngle > 90) {
      normalizedCurrentAngle -= 90;
    }
    double armLength = RobotConfig.Arm.kBaseArmLength + 0;
    double gravityCompensationFactor = 0.001 * armLength;
    double feedforward = gravityCompensationFactor * Math.cos(Math.toRadians(normalizedCurrentAngle));
    if((currentAngle < 90 && currentAngle > 0) || (currentAngle > 270 && currentAngle < 360)) {
      feedforward *= -1;
    }

    //Feedback
    double feedback = -1 * m_pivotPid.calculate(currentAngle, desiredAngle);

    SmartDashboard.putNumber("Arm Feedforward", feedforward);
    SmartDashboard.putNumber("Arm Feedback", feedback);
    double speed = MathUtil.clamp(feedforward + feedback, -1, 1);
    SmartDashboard.putNumber("Pivot Speed", speed);
    m_leftPivotMotor.set(speed);
  }

  /**
   * Checks if the given angle is in the bounds of the wrist.
   * 
   * @param angle the given angle
   * @return true if the given angle is in the bounds of the wrist.
   */
  private boolean isInPivotBounds(double angle) {
    return (angle > Arm.kMinPivotAngle && angle < Arm.kMaxPivotAngle);
  }

  public double limitPivotAngle(double angle) {
    return MathUtil.clamp(angle, Arm.kMinPivotAngle, Arm.kMaxPivotAngle);
  }

  // Intake

  public void runIntake(double intakeSpeed) {
    m_intakeMotor.set(m_intakeSpeed);
  }

  // Extension

  public double getExtensionLength() {
    return m_extensionMotor.getEncoder().getPosition();
  }

  private void setMotorExtensionLength(double extensionLength) {
    m_extensionMotor.getPIDController().setReference(extensionLength, CANSparkMax.ControlType.kPosition);
  }

  private boolean isExtensionMinSwitchActive() { 
    return m_minMagneticSwitch.get();
  }

  private boolean isExtensionMaxSwitchActive() {
    return m_maxMagneticSwitch.get();
  }

  public double limitExtensionLength(double length) {
    return MathUtil.clamp(length, Arm.kMinArmExtension, Arm.kMaxArmExtension);
  }

  /**
   * Checks if the given angle is in the bounds of the wrist.
   * 
   * @param angle the given angle
   * @return true if the given angle is in the bounds of the wrist.
   */
  private boolean isInExtensionBounds(double length) {
    return (length > Arm.kMinArmExtension && length < Arm.kMaxArmExtension);
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

  public double dynamicExtensionLengthLimit(double extensionLength) {

    if (getAbsolutePivotAngle() >= RobotConfig.Arm.kAngleForNoExtension) {
      return 0;
    }

    if (getAbsolutePivotAngle() > RobotConfig.Arm.kAngleForNoExtension && getAbsolutePivotAngle() < RobotConfig.Arm.kGreatestMaxExtensionAngle) {
      return maxGroundArmLength(getAbsolutePivotAngle(), m_wristMotor.getEncoder().getPosition(), RobotConfig.kHeightFromGround);
    }

    return extensionLength;
  }

  public void moveArm(double absoluteAngle, double intakeSpeed, double pivotAngle, double extensionLength) {
    this.m_wristAngle = absoluteAngle;
    this.m_intakeSpeed = intakeSpeed;
    this.m_pivotAngle = pivotAngle;
    this.m_extensionLength = extensionLength;
  }

  public void moveArm(ArmState state) {
    this.m_wristAngle = state.wristAngle;
    this.m_intakeSpeed = state.intakeSpeed;
    this.m_pivotAngle = state.armPivot;
    this.m_extensionLength = state.armLength;
  }

  @Override
  public void periodic() {
    if (isInWristBounds(m_wristMotor.getEncoder().getPosition()) || isInWristBounds(this.m_wristAngle)) {
      setAbsoluteWristAngle(this.m_wristAngle);
    } else {
      setAbsoluteWristAngle(m_wristMotor.getEncoder().getPosition());
    }

    runIntake(m_intakeSpeed);

    if((isInPivotBounds(getAbsolutePivotAngle()) && !isArmSwitchActive()) || isInPivotBounds(this.m_pivotAngle)) {
      setAbsolutePivotAngle(this.m_pivotAngle);
    } else if(m_leftPivotMotor.getOutputCurrent() > RobotConfig.Arm.kPivotAngleMaxAmperage) {
      m_leftPivotMotor.stopMotor();
    } else {
      setAbsolutePivotAngle(getAbsolutePivotAngle());
    }

    if(isExtensionMinSwitchActive() && m_extensionLimitSwitchOffset == 0) {
      //Rezero extension
      m_extensionLimitSwitchOffset = getExtensionLength();
      m_extensionMotor.getEncoder().setPosition(m_extensionLimitSwitchOffset);
      
      //Let it still move while resetting to leave the magnet zone
      if(isInExtensionBounds(m_extensionLength)) {
        setMotorExtensionLength(dynamicExtensionLengthLimit(m_extensionLength));
      } else {
        setMotorExtensionLength(getExtensionLength());
      }

    } else {
      if((!isExtensionMinSwitchActive() && !isExtensionMaxSwitchActive()) || isInExtensionBounds(m_extensionLength)) {
        setMotorExtensionLength(dynamicExtensionLengthLimit(m_extensionLength));
      } else {
        setMotorExtensionLength(getExtensionLength());
      }
    }
    SmartDashboard.putBoolean("Extension Switch", isExtensionMinSwitchActive());
    SmartDashboard.putNumber("Arm Extension", m_extensionMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Extension current", m_extensionMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Extension Max Switch", isExtensionMaxSwitchActive());
}
}
