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

    private boolean isFinished = true;

    public SwerveAutoBalance(SwerveSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_xPIDController = new PIDController(
            RobotConfig.SwerveConfig.kPTranslation, 
            RobotConfig.SwerveConfig.kITranslation, 
            RobotConfig.SwerveConfig.kDTranslation);
        m_yPIDController = new PIDController(
            RobotConfig.SwerveConfig.kPTranslation, 
            RobotConfig.SwerveConfig.kITranslation, 
            RobotConfig.SwerveConfig.kDTranslation);
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
        if ((Math.abs(m_drivetrainSubsystem.getPose().getX()) > 1.96 
        && (Math.abs(m_drivetrainSubsystem.getPose().getX()) < 2.04))) {
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
