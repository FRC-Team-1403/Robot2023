package team1403.lib.device.wpi;

import javax.print.attribute.standard.Compression;

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
        this.m_Compressor.setPressure(val);
        
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
        this.m_Compressor.setClosedLoopControl(value);
    }

    @Override
    public boolean getCompressorCurrentTooHighStickyFault() {
        return this.m_Compressor.getCompressorCurrentTooHighStickyFault();
    }

    @Override
    public boolean getCompressorShortedStickyFault() {
        return this.m_Compressor.getCompressorShortedStickyFault();
    }

    @Override
    public boolean getCompressorNotConnectedStickyFault() {
        return this.m_Compressor.getCompressorNotConnectedStickyFault();
    }

    @Override
    public void clearAllPCMStickyFaults() {
        this.m_Compressor.clearAllPCMStickyFaults();
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
        return this.m_Compressor.getClosedLoopControl();
    }


    
    
}
