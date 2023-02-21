package team1403.robot.chargedup.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.chargedup.RobotConfig;

public class SwerveAutoBalanceYaw extends CommandBase {
 
    private final SwerveSubsystem m_drivetrainSubsystem;
    private final PIDController m_xPIDController;
    private double velocity;

    public SwerveAutoBalanceYaw(SwerveSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_xPIDController = new PIDController(
            6.0, 
            RobotConfig.SwerveConfig.kITranslation, 
            0.5);
    }

    @Override
    public void execute() {
        velocity = m_xPIDController.calculate(m_)

        m_drivetrainSubsystem.drive(new ChassisSpeeds(xVelocity, yVelocity, 0),
            new Translation2d());
    }
}
