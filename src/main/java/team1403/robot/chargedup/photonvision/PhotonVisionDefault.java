package team1403.robot.chargedup.photonvision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import team1403.lib.util.SwerveDrivePoseEstimator;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

/**
 * The default command for photon vision.
 */
public class PhotonVisionDefault extends CommandBase {
  private SwerveSubsystem m_drivetrainSubsystem;
  private PhotonVisionSubsystem m_photonVisionSubsystem;

  private PhotonPoseEstimator m_photonPoseEstimator;
  private SwerveDrivePoseEstimator m_swervePoseEstimator;

  /**
   * Constructor for the default PhotonVision.
   *
   * @param drivetrainSubsystem   The Drivetrain subsystem used by PhotonVision
   *                              default.
   * @param photonVisionSubsystem The PhotonVision subsystem used by PhtonVision
   *                              default.
   */
  public PhotonVisionDefault(SwerveSubsystem drivetrainSubsystem,
      PhotonVisionSubsystem photonVisionSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;

    m_swervePoseEstimator = m_drivetrainSubsystem.getOdometer();

    m_photonPoseEstimator = m_photonVisionSubsystem.getPhotonPoseEstimator();

    addRequirements(m_photonVisionSubsystem);
  }

  @Override
  public void initialize() {
    m_photonPoseEstimator.setReferencePose(m_drivetrainSubsystem.getPose());
  }

  @Override
  public void execute() {
    Optional<EstimatedRobotPose> result = m_photonVisionSubsystem.getPhotonPose();
    if (result.isPresent()) {
      Pose2d photonPose = m_photonVisionSubsystem.getFieldLimelightBasedPose();
      double distanceFromCurrentMeasurment = photonPose.getTranslation()
          .getDistance(m_drivetrainSubsystem.getPose().getTranslation());
      if (Math.abs(distanceFromCurrentMeasurment) <= 1) {
        m_swervePoseEstimator.addVisionMeasurement(
            photonPose, Timer.getFPGATimestamp());
        m_photonPoseEstimator.setReferencePose(m_drivetrainSubsystem.getPose());

        SmartDashboard.putString("Vision Odometry", photonPose.toString());
      }
    }
  }
}
