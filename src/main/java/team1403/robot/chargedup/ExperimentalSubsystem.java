package team1403.robot.chargedup;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.wpi.WpiAnalogDevice;

public class ExperimentalSubsystem extends CougarSubsystem{
    private final WpiAnalogDevice photogate;
    private final Encoder throughBore;
    public ExperimentalSubsystem(String name, CougarLibInjectedParameters injectedParameters) {
        super(name, injectedParameters);
        photogate = new WpiAnalogDevice("photogate", 0);
        throughBore = new Encoder(1, 2);


        addDevice("photogate", photogate);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("photogateValues", photogate.getAnalogValue());

        SmartDashboard.putNumber("encoderValue", throughBore.get());
        SmartDashboard.putBoolean("encoderValue", throughBore.getDirection());
    }

}
