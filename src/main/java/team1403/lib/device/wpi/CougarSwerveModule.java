package team1403.lib.device.wpi;

import team1403.lib.device.SwerveModule;

public class CougarSwerveModule implements SwerveModule{
    private CougarSparkMax driveMotor;
    private CougarTalonFx steerMotor;

    public CougarSwerveModule(CougarSparkMax driveMotor, CougarTalonFx steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
    }

    @Override
    public void configEncoders() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void configDriveMotor() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void configSteerMotor() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getSteerAngle() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double angleError(double setpoint) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void set(double driveVoltage, double steerAngle) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getAbsoluteEncoderAbsoluteAngle() {
        // TODO Auto-generated method stub
        return 0;
    }
}
