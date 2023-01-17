package team1403.lib.device.wpi;

import edu.wpi.first.wpilibj.Compressor;
import team1403.lib.device.CougarCompressor;

public class WPICompressor implements CougarCompressor{

    private String m_name;
    private Compressor m_Compressor;
    
    public WPICompressor(String name, Compressor compressor) {
        this.m_name = name;
        this.m_Compressor = compressor;
    }


    @Override
    public String getName() {
        return this.m_name;
    }

    @Override
    public void stop() {
        this.m_Compressor.disable();
    }

    @Override
    public double getPressureSwitchValue() {
        return this.m_Compressor.getPressure();
    }

    @Override
    public void setPressure(int val) {
        //Set a constant for the pressure of the compressor.
        
    }

    @Override
    public boolean isEnabled() {
        return this.m_Compressor.isEnabled();
    }

    @Override
    public double getCurrent() {
        return this.m_Compressor.getCurrent();
    }

    @Override
    public void setClosedLoopControl(boolean value) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean getCompressorCurrentTooHighStickyFault() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getCompressorShortedStickyFault() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getCompressorNotConnectedStickyFault() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void clearAllPCMStickyFaults() {
        // TODO Auto-generated method stub
        
    }


    @Override
    public void start(String type) {
        if (type.equals("Digital")) {
            this.m_Compressor.enableDigital();
        }
        else if (type.equals("Analog")) {
            this.m_Compressor.enableAnalog(getPressureSwitchValue(), getPressureSwitchValue());
        }
        else if (type.equals("Hybrid")) {
            this.m_Compressor.enableHybrid(getPressureSwitchValue(), getPressureSwitchValue());
        }        
    }


    @Override
    public boolean getClosedLoopControl() {
        // TODO Auto-generated method stub
        return false;
    }


    
    
}
