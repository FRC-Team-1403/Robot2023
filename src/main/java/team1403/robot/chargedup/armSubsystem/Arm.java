 
package team1403.robot.chargedup.armSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.AdvancedMotorController;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.chargedup.RobotConfig;
import team1403.robot.chargedup.RobotConfig.CanBus;
import team1403.robot.chargedup.RobotConfig.RioPorts;

public class Arm extends CougarSubsystem {

    /* TODO
     * Code encoder as secondary safety measure for anglation
     */

     private final RobotConfig.Arm m_armConfig;
     private final CougarSparkMax m_telescopicMotor;
     private final CougarSparkMax m_leftAngledMotor;
     private final CougarSparkMax m_rightAngledMotor;
     private final WpiLimitSwitch m_frontArmSwitch;
     private final WpiLimitSwitch m_backArmSwitch;
     private final PIDController m_pidArmRotation;
     private final PIDController m_pidWristRotation;

    public Arm(CougarLibInjectedParameters injectedParameters, RobotConfig robotConfig) {
       super("TelescopicArm", injectedParameters);
       this.m_armConfig = robotConfig.arm;

       CanBus can = robotConfig.canBus; //This is specfic to SparkCanMax motor
       var ports = robotConfig.ports;

       var logger = getLogger(); //after game, to see what robot did
       
       m_telescopicMotor = CougarSparkMax.makeBrushless("telescopic", can.telescopicArmMotor, 
                           Type.kHallSensor, getLogger());

       m_leftAngledMotor = CougarSparkMax.makeBrushless("leftAngledArmMotor", can.leftAngledArmMotor, 
                           null, logger);

       m_rightAngledMotor = CougarSparkMax.makeBrushless("rightAngledArmMotor", can.rightAngledArmMotor, 
                null, logger);

       m_frontArmSwitch = new WpiLimitSwitch("frontrSwitch", ports.exampleRailForwardLimitSwitch);

       m_backArmSwitch = new WpiLimitSwitch("backSwitch", ports.exampleRailReverseLimitSwitch);

       m_rightAngledMotor.setInverted(true);

       m_rightAngledMotor.follow((AdvancedMotorController) m_leftAngledMotor);

       m_pidArmRotation = new PIDController(0, 0, 0);
       m_pidWristRotation = new PIDController(0, 0, 0);
    }

        public 

        public boolean isFrontSwitch() {
            return m_frontArmSwitch.isTriggered();
        }

        public boolean isBackSwitch() {
            return m_backArmSwitch.isTriggered();
        }
     
        public void setTelescopicMotorSpeed(double speed) {
            m_telescopicMotor.setSpeed(speed);
        }
     
        public void setAngledMotorSpeed(double speed) {
            m_leftAngledMotor.setSpeed(speed);
        }
     
        public RobotConfig.Arm getArmConfig() {
            return m_armConfig;
        }
     
            //TODO will change such that we do not have to do * 42 (will be built-in)
        public double getArmAngle() {
            return m_leftAngledMotor.getEmbeddedEncoder().getPositionTicks() * 42;
        }
     
        public double getCurrentAmps() {
            return m_leftAngledMotor.getEmbeddedCurrentSensor().getAmps();
        }
    }