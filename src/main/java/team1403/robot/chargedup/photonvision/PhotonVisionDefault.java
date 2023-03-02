package team1403.robot.chargedup.photonvision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import team1403.lib.util.SwerveDrivePoseEstimator;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

public class PhotonVisionDefault extends CommandBase {
    private SwerveSubsystem m_drivetrainSubsystem;
    private PhotonVisionSubsystem m_photonVisionSubsystem;

    private Optional<EstimatedRobotPose> photonPose;
    
    private PhotonPoseEstimator photonPoseEstimator;
    private SwerveDrivePoseEstimator swervePoseEstimator;

    public PhotonVisionDefault(SwerveSubsystem drivetrainSubsystem,
                               PhotonVisionSubsystem photonVisionSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_photonVisionSubsystem = photonVisionSubsystem;

        swervePoseEstimator = m_drivetrainSubsystem.getOdometer();

        photonPoseEstimator = m_photonVisionSubsystem.getPhotonPoseEstimator();
        photonPose = m_photonVisionSubsystem.getPhotonPose();

        addRequirements(m_photonVisionSubsystem);
    }

    @Override
    public void initialize() {
        Rotation2d gyroscopeRotation = m_drivetrainSubsystem.getGyroscopeRotation();
        SwerveModulePosition[] swerveModulePositions = m_drivetrainSubsystem.getModulePositions();

        swervePoseEstimator.resetPosition(gyroscopeRotation, swerveModulePositions, photonPose.get().estimatedPose.toPose2d());
    }

    @Override
    public void execute() {
        Rotation2d gyroscopeRotation = m_drivetrainSubsystem.getGyroscopeRotation();
        SwerveModulePosition[] swerveModulePositions = m_drivetrainSubsystem.getModulePositions();

        if(m_photonVisionSubsystem.getPhotonPose().isPresent()) {
            photonPose = photonPoseEstimator.update();
            if ((Math.abs(photonPose.get().estimatedPose.getX() - m_drivetrainSubsystem.getOdometryValue().getX()) < 1)
                && ((Math.abs(photonPose.get().estimatedPose.getY() - m_drivetrainSubsystem.getOdometryValue().getY()) < 1)))

            swervePoseEstimator.addVisionMeasurement(photonPose.get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
        }

        swervePoseEstimator.update(gyroscopeRotation, swerveModulePositions);

        SmartDashboard.putString("Swerve Odometry", swervePoseEstimator.getEstimatedPosition().toString());
        SmartDashboard.putString("Vision Odometry", photonPose.get().estimatedPose.toString());
        SmartDashboard.putString("Combined Odometry", swervePoseEstimator.getEstimatedPosition().toString());
    }
}
