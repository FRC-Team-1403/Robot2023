package team1403.robot.chargedup.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.chargedup.RobotConfig;

public class SwerveAutoBalanceYaw extends CommandBase {
 
    private final SwerveSubsystem m_drivetrainSubsystem;
    private final ProfiledPIDController m_xPIDController;
    private double velocity;
    private double previousRollValue;
    private double rollValue;
    private double rollSetpoint;

    // Roll velocity is in degrees per 20 milliseconds
    private double rollVelocity;

    public SwerveAutoBalanceYaw(SwerveSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_xPIDController = new ProfiledPIDController(
            0.1, 
            RobotConfig.Swerve.kITranslation, 
            0,
            new TrapezoidProfile.Constraints(200, 200));
        m_drivetrainSubsystem.resetOdometry();
        previousRollValue = 0;
        rollValue = 0;
        rollSetpoint = 0;
    }

    @Override
    public void execute() {
      rollValue = m_drivetrainSubsystem.getGyroRoll();

      rollVelocity = (rollValue - previousRollValue) / 0.2;

      velocity = m_xPIDController.calculate(rollValue, 0);

      SmartDashboard.putNumber("roll velocity", velocity);

      m_drivetrainSubsystem.drive(new ChassisSpeeds(velocity, 0, 0),
          new Translation2d());
  
      previousRollValue = m_drivetrainSubsystem.getGyroRoll();
    }

    @Override
    public boolean isFinished() {
        return rollValue == 0 && Math.abs(rollVelocity) > 3;
    }

    @Override
    public void end(boolean interupted) {
        m_drivetrainSubsystem.setXModeEnabled(true);
    }
}
