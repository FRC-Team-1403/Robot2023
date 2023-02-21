package team1403.robot.chargedup.arm;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig;

public class Arm_Subsystem extends CougarSubsystem{
  private final CougarSparkMax m_wristMotor;


  public Arm_Subsystem(CougarLibInjectedParameters injectedParameters) {
    super("Arm", injectedParameters);
    CougarLogger logger = getLogger();
    m_wristMotor = CougarSparkMax.makeBrushless("Wrist Motor", RobotConfig.CanBus.wristMotor,  Type.kHallSensor, logger);
  }

  private void configWristMotor() {
    SparkMaxPIDController controller = m_wristMotor.getPIDController();
    controller.setP(RobotConfig.Arm.kPWristMotor);
    controller.setI(RobotConfig.Arm.kIWristMotor);
    controller.setD(RobotConfig.Arm.kDWristMotor);
  }

  private void setAbsoluteWristAngle(double absoluteWristAngle) {
    
  }

  @Override
  public void periodic() {
    m_wristMotor.set(-0.10);
  }

  
}
