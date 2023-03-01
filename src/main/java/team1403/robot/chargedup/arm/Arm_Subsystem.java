package team1403.robot.chargedup.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
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
  private final CANSparkMax m_telescopicMotor;

  // Setpoints
  private double m_wristAngle;
  private double m_pivotAngle;
  private double m_intakeSpeed;
  private double m_extension;

  public Arm_Subsystem(CougarLibInjectedParameters injectedParameters) {
    super("Arm", injectedParameters);
    CougarLogger logger = getLogger();

    m_wristMotor = CougarSparkMax.makeBrushless("Wrist Motor", RobotConfig.CanBus.wristMotor, Type.kHallSensor, logger);
    m_wristAbsoluteEncoder = new DutyCycleEncoder(RobotConfig.RioPorts.kWristAbsoluteEncoder);

    m_leftPivotMotor = new CANSparkMax(RobotConfig.CanBus.leftPivotMotor, MotorType.kBrushless);
    m_rightPivotMotor = new CANSparkMax(RobotConfig.CanBus.rightPivotMotor, MotorType.kBrushless);
    m_armAbsoluteEncoder = new AnalogEncoder(RobotConfig.RioPorts.kArmAbsoluteEncoder);

    m_intakeMotor = new CANSparkMax(RobotConfig.CanBus.wheelIntakeMotor, MotorType.kBrushed);

    m_telescopicMotor = new CANSparkMax(RobotConfig.CanBus.telescopicArmMotor, MotorType.kBrushless);

    m_maxArmLimitSwitch = new WpiLimitSwitch("maxArmLimitSwitch",
    RobotConfig.RioPorts.kArmLimitSwitch);

    m_intakeSpeed = 0;
    configWristMotor();
    configEncoders();

    // TODO, tune feedforward with arm pivot
    // TODO Constraints
    m_pivotPid = new PIDController(0.025, RobotConfig.Arm.kIArmPivot, RobotConfig.Arm.kDArmPivot);

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
  public void extendMotor(double extensionSpeed) {
    m_telescopicMotor.set(extensionSpeed);
  }

  public void move(double absoluteAngle, double intakeSpeed, double pivotAngle, double extensionSpeed) {
    this.m_wristAngle = absoluteAngle;
    this.m_intakeSpeed = intakeSpeed;
    this.m_pivotAngle = pivotAngle;
    this.m_extension = extensionSpeed;
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



    extendMotor(m_extension);

    SmartDashboard.putNumber("Wrist Setpoint", m_wristAngle);
    SmartDashboard.putNumber("Arm Absolute", getAbsolutePivotAngle());
    SmartDashboard.putNumber("Arm Relative", m_leftPivotMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("Limit Switch", isArmSwitchActive());

  }

}
