package team1403.robot.chargedup;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiAnalogDevice;
import team1403.lib.device.wpi.WpiLimitSwitch;

public class ExperimentalSubsystem extends CougarSubsystem{
    private final WpiLimitSwitch limitSwitch;
    private final WpiAnalogDevice potentiometer;
    private final DutyCycleEncoder grayhill;
    public ExperimentalSubsystem(String name, CougarLibInjectedParameters injectedParameters) {
        super(name, injectedParameters);
        grayhill = new DutyCycleEncoder(0);
        limitSwitch = new WpiLimitSwitch("limitSwitch", 0);
        potentiometer = new WpiAnalogDevice("potentiometer", 0);
      }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limit Switch Value", limitSwitch.get());
        SmartDashboard.putNumber("Potentiometer Value", potentiometer.getAnalogValue());
        SmartDashboard.putNumber("Encoder Value", grayhill.get());
    }

}
