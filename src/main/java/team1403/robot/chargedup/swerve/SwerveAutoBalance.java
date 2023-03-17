package team1403.robot.chargedup.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.chargedup.RobotConfig;

public class SwerveAutoBalance extends CommandBase {
 
    private final SwerveSubsystem m_drivetrainSubsystem;

    private final PIDController m_xPIDController;
    private final PIDController m_yPIDController;

    private double xVelocity;
    private double yVelocity;

    public SwerveAutoBalance(SwerveSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_xPIDController = new PIDController(
            RobotConfig.Swerve.kPTranslation, 
            RobotConfig.Swerve.kITranslation, 
            RobotConfig.Swerve.kDTranslation);
        m_yPIDController = new PIDController(
            RobotConfig.Swerve.kPTranslation, 
            RobotConfig.Swerve.kITranslation, 
            RobotConfig.Swerve.kDTranslation);
    }

    @Override
    public void execute() {
        xVelocity = m_xPIDController.calculate(m_drivetrainSubsystem.getPose().getX(), 2);
        yVelocity = m_yPIDController.calculate(m_drivetrainSubsystem.getPose().getY(), 0);

        m_drivetrainSubsystem.drive(new ChassisSpeeds(xVelocity, yVelocity, 0),
            new Translation2d());
    }

    @Override
    public boolean isFinished() {
        if (m_drivetrainSubsystem.getPose().getX() > 1.96 
        && (m_drivetrainSubsystem.getPose().getX() < 2.04)) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(
            new ChassisSpeeds(0, 0, 0.1), new Translation2d());
    }
}
