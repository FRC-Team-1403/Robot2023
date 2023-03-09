package team1403.robot.chargedup.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.AdvancedMotorController.CougarIdleMode;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig;
import team1403.robot.chargedup.RobotConfig.Arm;

/**
 * Class creating the arm subsystem.
 * 
 */
public class ArmSubsystem extends CougarSubsystem {
  // Wrist
  private final CougarSparkMax m_wristMotor;
  private final DutyCycleEncoder m_wristAbsoluteEncoder;

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
  private double m_wristAngleSetpoint;
  private double m_pivotAngleSetpoint;
  private double m_intakeSpeedSetpoint;
  private double m_extensionLengthSetpoint;

  /**
   * Initializing the arn subsystem.
   *
   * @param injectedParameters Cougar injected parameters.
   */
  public ArmSubsystem(CougarLibInjectedParameters injectedParameters) {
    super("Arm", injectedParameters);
    CougarLogger logger = getLogger();

    m_wristMotor = CougarSparkMax.makeBrushless("Wrist Motor",
        RobotConfig.CanBus.wristMotor, Type.kHallSensor, logger);
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

    this.m_pivotAngleSetpoint = getAbsolutePivotAngle();
    this.m_wristAngleSetpoint = getRelativeWristAngle();
    this.m_extensionLengthSetpoint = getExtensionLength();

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
      double wristAngle = getRelativeWristAngle();
      m_wristMotor.getEncoder().setPosition(wristAngle);
      m_wristAngleSetpoint = wristAngle;
    }).start();

    // Telescopic encoders
    m_extensionMotor.getEncoder().setPositionConversionFactor(
        RobotConfig.Arm.kExtensionConversionFactor);

    // Arm encoders
    m_leftPivotMotor.getEncoder().setPositionConversionFactor(1.53285964552);
    m_leftPivotMotor.getEncoder().setPosition(getAbsolutePivotAngle());
  }

  private void configWristMotor() {
    // Wrist
    final SparkMaxPIDController wristController = m_wristMotor.getPIDController();
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setInverted(false);
    m_wristMotor.enableVoltageCompensation(12);
    m_wristMotor.setSmartCurrentLimit(20);
    m_wristMotor.setRampRate(0.25);

    wristController.setP(0.05);
    wristController.setI(Arm.kIWristMotor);
    wristController.setD(0);
    wristController.setFeedbackDevice((MotorFeedbackSensor) m_wristMotor.getEncoder());
    wristController.setPositionPIDWrappingEnabled(false);

    // Pivot
    m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
    m_leftPivotMotor.enableVoltageCompensation(12);
    m_leftPivotMotor.setSmartCurrentLimit(25);
    m_rightPivotMotor.follow(m_leftPivotMotor, true);

    //intake
    m_wristMotor.setIdleMode(CougarIdleMode.BRAKE);

    // Extension
    final SparkMaxPIDController extensionController = m_extensionMotor.getPIDController();
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

  public double getWristAngleSetpoint() {
    return m_wristAngleSetpoint;
  }

  public double getPivotAngleSetpoint() {
    return m_pivotAngleSetpoint;
  }

  public double getIntakeSpeedSetpoint() {
    return m_intakeSpeedSetpoint;
  }

  public double getExtensionLengthSetpoint() {
    return m_extensionLengthSetpoint;
  }

  // Wrist Methods

  /**
   * Gets the absolute wrist encoder value.
   *
   * @return The absolute encoder value of the wrist.
   */
  public double getRelativeWristAngle() {
    double value = (m_wristAbsoluteEncoder.getAbsolutePosition() * 360) + RobotConfig.Arm.kAbsoluteWristOffset;

    if (value < 0) {
      value += 360;
    }
    if (value > 360) {
      value -= 360;
    }
    return value;
  }

  private void setAbsoluteWristAngle(double absoluteWristAngle) {
    m_wristMotor.getPIDController().setReference(absoluteWristAngle,
        CANSparkMax.ControlType.kPosition);
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

  /**
   * The absolute pivot encoder value.
   *
   * @return The encoder value of the pivot encoder.
   */
  public double getAbsolutePivotAngle() {
    double value = (m_armAbsoluteEncoder.getAbsolutePosition() * 360) + RobotConfig.Arm.kAbsolutePivotOffset;

    if (value < 0) {
      value += 360;
    }
    if (value > 360) {
      value -= 360;
    }
    return value;
  }

  private void setAbsolutePivotAngle(double desiredAngle) {
    // Feedforward
    double currentAngle = getAbsolutePivotAngle();
    double normalizedCurrentAngle = currentAngle;
    while (normalizedCurrentAngle > 90) {
      normalizedCurrentAngle -= 90;
    }
    double armLength = RobotConfig.Arm.kBaseArmLength + getExtensionLength();
    double gravityCompensationFactor = 0.0015 * armLength;
    double feedforward = gravityCompensationFactor
        * Math.cos(Math.toRadians(normalizedCurrentAngle));
    if ((currentAngle < 90 && currentAngle > 0) || (currentAngle > 270 && currentAngle < 360)) {
      feedforward *= -1;
    }

    // Feedback
    double feedback = -1 * m_pivotPid.calculate(currentAngle, desiredAngle);

    if(isArmSwitchActive()) {
      feedforward = 0;
    }

    SmartDashboard.putNumber("Arm Feedforward", feedforward);
    SmartDashboard.putNumber("Arm Feedback", feedback);
    double speed = MathUtil.clamp(feedforward + feedback, -1, 1);
    m_leftPivotMotor.set(speed);
  }

  /**
   * Checks if the given angle is in the bounds of the wrist.
   *
   * @param angle the given angle
   * @return true if the given angle is in the bounds of the wrist.
   */
  private boolean isInPivotBounds(double angle) {
    return (angle >= Arm.kMinPivotAngle && angle <= Arm.kMaxPivotAngle);
  }

  public double limitPivotAngle(double angle) {
    return MathUtil.clamp(angle, Arm.kMinPivotAngle, Arm.kMaxPivotAngle);
  }

  // Intake

  public void runIntake(double intakeSpeed) {
    m_intakeMotor.set(m_intakeSpeedSetpoint);
  }

  // Extension

  public double getExtensionLength() {
    return m_extensionMotor.getEncoder().getPosition();
  }

  private void setMotorExtensionLength(double extensionLength) {
    m_extensionMotor.getPIDController().setReference(
        extensionLength, CANSparkMax.ControlType.kPosition);
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
   * Calculates the theoretical max arm length of the arm given the arm angle.
   *
   * @param absoluteArmAngle arm angle relative to ground
   * @return the theoretical arm length
   */
  public double theoreticalExtensionLength(double absoluteArmAngle, double height) {
    return (height / Math.cos(Math.toRadians(270 - absoluteArmAngle)))
        - RobotConfig.Arm.kExtensionOffset - RobotConfig.Arm.kBaseArmLength;
  }

  /**
   * Dynamically limits the extensions of arm so that it doesn't hit the ground.
   *
   * @param extensionLength extension length.
   * @return the arm length.
   */
  public double dynamicExtensionLimit(double extensionLength) {
    if (getAbsolutePivotAngle() >= RobotConfig.Arm.kFrameClearanceAngle) {
      return 0;
    } else if (getAbsolutePivotAngle() > RobotConfig.Arm.kHorizonAngle
        && getAbsolutePivotAngle() <= RobotConfig.Arm.kFrameClearanceAngle) {
      double maxLength = theoreticalExtensionLength(
          getAbsolutePivotAngle(), RobotConfig.kHeightFromGround);
      return MathUtil.clamp(extensionLength, 0, maxLength);
    } else if (getAbsolutePivotAngle() > RobotConfig.Arm.kFrameClearanceAngle
          && getAbsolutePivotAngle() <= RobotConfig.Arm.kFrameAngle) {
      double maxLength = -(8.355/4) * (getAbsolutePivotAngle()-229)  ;
      return MathUtil.clamp(extensionLength, 0, maxLength);
    }
    return extensionLength;
  }

  /**
   * Sets values that the arm uses.
   *
   * @param wristAngle   the wrist absolute angle.
   * @param intakeSpeed     intake speed.
   * @param pivotAngle      the pivot angle.
   * @param extensionLength the extension length.
   */
  public void moveArm(double wristAngle, double intakeSpeed,
      double pivotAngle, double extensionLength) {
    this.m_wristAngleSetpoint = wristAngle;
    this.m_intakeSpeedSetpoint = intakeSpeed;
    this.m_pivotAngleSetpoint = pivotAngle;
    this.m_extensionLengthSetpoint = extensionLength;
  }

  /**
   * Sets arm values based off of the ArmState class.
   *
   * @param state ArmState class.
   */
  public void moveArm(ArmState state) {
    this.m_wristAngleSetpoint = state.wristAngle;
    this.m_intakeSpeedSetpoint = state.intakeSpeed;
    this.m_pivotAngleSetpoint = state.armPivot;
    this.m_extensionLengthSetpoint = state.armLength;
  }

  /** Returns whether the arm is at the current setpoint.
   *
   * @return true if the arm is at the current setpoint.
   */
  public boolean isAtSetpoint() {
    double currentPivotAngle = getAbsolutePivotAngle();
    double currentWristAngle = getRelativeWristAngle();
    double currentExtensionLength = getExtensionLength();

    if(Math.abs(currentPivotAngle - this.m_pivotAngleSetpoint) > 2) {
      return false;
    }

    if(Math.abs(currentWristAngle - this.m_wristAngleSetpoint) > 2) {
      return false;
    } 

    if(Math.abs(currentExtensionLength - this.m_extensionLengthSetpoint) > 0.5) {
      return false;
    }

    return true;
  }

  @Override
  public void periodic() {
    // Wrist
    if (isInWristBounds(m_wristMotor.getEncoder().getPosition())
        || isInWristBounds(this.m_wristAngleSetpoint)) {
      setAbsoluteWristAngle(this.m_wristAngleSetpoint);
    } else {
      setAbsoluteWristAngle(m_wristMotor.getEncoder().getPosition());
    }

    // Intake
    runIntake(m_intakeSpeedSetpoint);

    // Pivot
    if ((isInPivotBounds(getAbsolutePivotAngle()) && !isArmSwitchActive())
        || isInPivotBounds(this.m_pivotAngleSetpoint)) {
      setAbsolutePivotAngle(this.m_pivotAngleSetpoint);
    } else if (m_leftPivotMotor.getOutputCurrent() > RobotConfig.Arm.kPivotAngleMaxAmperage) {
      m_leftPivotMotor.stopMotor();
    } else {
      setAbsolutePivotAngle(getAbsolutePivotAngle());
    }

    // Extension
    double limitedExtension = dynamicExtensionLimit(m_extensionLengthSetpoint);
    SmartDashboard.putNumber("Limited length", limitedExtension);

    // TODO if condition to change setpoint to limit
    m_extensionLengthSetpoint = limitedExtension;

    if (isExtensionMinSwitchActive() && m_extensionLimitSwitchOffset == 0) {
      // Rezero extension
      m_extensionLimitSwitchOffset = getExtensionLength();
      m_extensionMotor.getEncoder().setPosition(m_extensionLimitSwitchOffset);

      // Let it still move while resetting to leave the magnet zone
      if (isInExtensionBounds(limitedExtension)) {
        setMotorExtensionLength(dynamicExtensionLimit(limitedExtension));
      } else {
        setMotorExtensionLength(getExtensionLength());
      }
    } else {
      if ((!isExtensionMinSwitchActive() && !isExtensionMaxSwitchActive())
          || isInExtensionBounds(limitedExtension)) {
        setMotorExtensionLength(dynamicExtensionLimit(limitedExtension));
      } else {
        setMotorExtensionLength(getExtensionLength());
      }
    }

    // Track Values
    SmartDashboard.putNumber("Wrist Angle", getRelativeWristAngle());
    SmartDashboard.putNumber("Pivot Angle", getAbsolutePivotAngle());
    SmartDashboard.putNumber("Extension Length", getExtensionLength());

    SmartDashboard.putNumber("WristSetpoint", getWristAngleSetpoint());
    SmartDashboard.putNumber("Pivot Setpoint", getPivotAngleSetpoint());
    SmartDashboard.putNumber("Extension Setpoint", getExtensionLengthSetpoint());
    SmartDashboard.putBoolean("minExtension", isExtensionMinSwitchActive());
    SmartDashboard.putNumber("RAW VALUE", m_wristAbsoluteEncoder.getAbsolutePosition());
    SmartDashboard.putBoolean("Arm Switch??", isArmSwitchActive());
  }

  public CougarSparkMax getWristMotor() {
    return m_wristMotor;
  }

  public CANSparkMax getLeftPivotMotor() {
    return m_leftPivotMotor;
  }


  public CANSparkMax getIntakeMotor() {
    return m_intakeMotor;
  }

  public CANSparkMax getExtensionMotor() {
    return m_extensionMotor;
  }
}