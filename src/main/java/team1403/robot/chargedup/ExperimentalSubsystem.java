package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.wpi.WpiAnalogDevice;

public class ExperimentalSubsystem extends CougarSubsystem{
    private final WpiAnalogDevice photogate;
    public ExperimentalSubsystem(String name, CougarLibInjectedParameters injectedParameters) {
        super(name, injectedParameters);
        photogate = new WpiAnalogDevice("photogate", 0);

        addDevice("photogate", photogate);
    }

    @Override
    public void periodic() {
        SmartDashboard.getNumber("photogateValues", photogate.getAnalogValue());
    }

}
