package team1403.lib.device.wpi;

import com.kauailabs.navx.frc.AHRS;

import team1403.lib.device.GyroscopeDevice;
import edu.wpi.first.wpilibj.SPI;

public class NavxAhrs implements GyroscopeDevice {
    private final String name;
    private final AHRS navx;
    public NavxAhrs(String name) {
        this.name = name;
        this.navx = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void reset() {
        navx.reset();        
    }

    @Override
    public double getRawAngle() {
        return navx.getAngle() - navx.getAngleAdjustment();
    }

    @Override
    public double getAngle() {
        return navx.getAngle();
    }

    @Override
    public double getAngularVelocity() {
        return navx.getRate();
    }

    @Override
    public void setAngleOffset(double angleOffset) {
        navx.setAngleAdjustment(angleOffset); 
    }

    @Override
    public double getAngleOffset() {
        return navx.getAngleAdjustment();
    }

}