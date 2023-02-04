package team1403.robot.chargedup;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.wpi.WpiAnalogDevice;

public class ExperimentalSubsystem extends CougarSubsystem{
    private final DigitalInput photogate;
    private final Encoder throughBore;
    public ExperimentalSubsystem(String name, CougarLibInjectedParameters injectedParameters) {
        super(name, injectedParameters);
        photogate = new DigitalInput(0);
        throughBore = new Encoder(0,0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("photogateValues", photogate.get());

        SmartDashboard.putNumber("encoderValue", throughBore.get());
        SmartDashboard.putBoolean("encoderValue", throughBore.getDirection());
    }

}
