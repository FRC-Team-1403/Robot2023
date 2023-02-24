package team1403.robot.chargedup.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.AdvancedMotorController;
import team1403.lib.device.AnalogDevice;
import team1403.lib.device.DeviceFactory;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiAnalogDevice;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig.Arm;
import team1403.robot.chargedup.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class Arm_Subsystem extends CougarSubsystem{
  //Wrist
  private final CougarSparkMax m_wristMotor;
  private final DutyCycleEncoder m_wristAbsoluteEncoder;
  private double m_absoluteWristOffset = 30;

  //Arm
  private final CougarSparkMax m_leftPivotMotor;
  private final CougarSparkMax m_rightPivotMotor;
  private final AnalogDevice m_armAbsoluteEncoder;
  private double m_absoluteArmOffset = 0;


  //Setpoints
  private double m_wristAngle;
  private double m_armAngle;



  public Arm_Subsystem(CougarLibInjectedParameters injectedParameters) {
    super("Arm", injectedParameters);
    CougarLogger logger = getLogger();

    m_wristMotor = CougarSparkMax.makeBrushless("Wrist Motor", RobotConfig.CanBus.wristMotor,  Type.kHallSensor, logger);
    m_leftPivotMotor = CougarSparkMax.makeBrushless("Left Arm Motor", RobotConfig.CanBus.leftPivotMotor, Type.kHallSensor, logger);
    m_rightPivotMotor = CougarSparkMax.makeBrushless("Right Arm Motor", RobotConfig.CanBus.rightPivotMotor, Type.kHallSensor, logger);
    m_wristAbsoluteEncoder = new DutyCycleEncoder(RobotConfig.RioPorts.kWristAbsoluteEncoder);
    m_armAbsoluteEncoder = new WpiAnalogDevice("Arm Absolute Encoder", RobotConfig.RioPorts.kArmAbsoluteEncoder);

    m_rightPivotMotor.setInverted(true);
    m_rightPivotMotor.follow((AdvancedMotorController) m_leftPivotMotor);

    configWristMotor();
    configEncoders();
  }

  private void configEncoders() {
    //Wrist encoders
    m_wristMotor.getEncoder().setPositionConversionFactor(90.0/100);
    new Thread(() -> {
        try {
          Thread.sleep(2000);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      double angle = getWristAbsoluteAngle();
      m_wristMotor.getEncoder().setPosition(angle);
      m_wristAngle = angle;
    }).start();
  }
  
  private void configWristMotor() {
    SparkMaxPIDController controller = m_wristMotor.getPIDController();
    m_wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_wristMotor.setInverted(false);
    m_wristMotor.enableVoltageCompensation(12);
    m_wristMotor.setSmartCurrentLimit(20);
    m_wristMotor.setRampRate(0.25);

    controller.setP(Arm.kPWristMotor);
    controller.setI(Arm.kIWristMotor);
    controller.setD(Arm.kDWristMotor);
    controller.setFeedbackDevice((MotorFeedbackSensor) m_wristMotor.getEncoder());
    controller.setPositionPIDWrappingEnabled(false);
  }

  private void setAbsoluteWristAngle(double absoluteWristAngle) {
    m_wristMotor.getPIDController().setReference(absoluteWristAngle, CANSparkMax.ControlType.kPosition);
  }

  public double getWristAbsoluteAngle() {
    double value = (m_wristAbsoluteEncoder.getAbsolutePosition() * 360) + m_absoluteWristOffset;

    if(value < 0) {
      value += 360;
    }
    if(value > 360) {
      value -= 360;
    }
    return value;
  }

  public double absoluteWristAngle(double desiredWristAngle, double desiredArmAngle,
      double currentWristAngle) {
    return currentWristAngle - (desiredArmAngle - 180) - 180 + desiredWristAngle;
  }

  public void move(double absoluteAngle) {
    this.m_wristAngle = absoluteAngle;
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

  @Override
  public void periodic() {
    if(isInWristBounds(m_wristMotor.getEncoder().getPosition()) || isInWristBounds(this.m_wristAngle)) {
      setAbsoluteWristAngle(this.m_wristAngle);
    } else {
      setAbsoluteWristAngle(m_wristMotor.getEncoder().getPosition());
    }
    SmartDashboard.putBoolean("Wrist Connection", m_wristAbsoluteEncoder.isConnected());
    SmartDashboard.putNumber("Wrist Relative Angle", m_wristMotor.getEmbeddedEncoder().getPositionValue());
    SmartDashboard.putNumber("Wrist Absolute Angle", getWristAbsoluteAngle());
    SmartDashboard.putNumber("Relative Conversion Factor", m_wristMotor.getEncoder().getPositionConversionFactor());
    
  }
  
}
