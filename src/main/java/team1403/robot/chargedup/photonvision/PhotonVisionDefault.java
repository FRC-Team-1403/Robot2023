package team1403.robot.chargedup.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
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

  private PhotonPoseEstimator photonPoseEstimator;
  private SwerveDrivePoseEstimator swervePoseEstimator;

  public PhotonVisionDefault(SwerveSubsystem drivetrainSubsystem,
      PhotonVisionSubsystem photonVisionSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;

    swervePoseEstimator = m_drivetrainSubsystem.getOdometer();

    photonPoseEstimator = m_photonVisionSubsystem.getPhotonPoseEstimator();

    addRequirements(m_photonVisionSubsystem);
  }

  @Override
  public void initialize() {
    photonPoseEstimator.setReferencePose(m_drivetrainSubsystem.getOdometryValue());
    Optional<EstimatedRobotPose> result = m_photonVisionSubsystem.getPhotonPose();
    if (result.isPresent() && swervePoseEstimator.getEstimatedPosition().equals(new Pose2d())) {
      EstimatedRobotPose photonPose = result.get();
      swervePoseEstimator.addVisionMeasurement(
          photonPose.estimatedPose.toPose2d(), photonPose.timestampSeconds);
      SmartDashboard.putString("Odometry", photonPose.estimatedPose.toPose2d().toString());
      swervePoseEstimator.resetPosition(m_drivetrainSubsystem.getGyroscopeRotation(),
          m_drivetrainSubsystem.getModulePositions(), photonPose.estimatedPose.toPose2d());
    }

  }

  @Override
  public void execute() {
    Optional<EstimatedRobotPose> result = m_photonVisionSubsystem.getPhotonPose();
    if (result.isPresent()) {
      EstimatedRobotPose photonPose = result.get();
      swervePoseEstimator.addVisionMeasurement(
          photonPose.estimatedPose.toPose2d(), photonPose.timestampSeconds);
      SmartDashboard.putString("Odometry", photonPose.estimatedPose.toPose2d().toString());
      photonPoseEstimator.setReferencePose(m_drivetrainSubsystem.getOdometryValue());
    }
  }
}
