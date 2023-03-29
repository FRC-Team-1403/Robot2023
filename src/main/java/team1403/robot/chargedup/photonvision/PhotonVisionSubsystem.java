package team1403.robot.chargedup.photonvision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.chargedup.RobotConfig;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.fasterxml.jackson.databind.util.ArrayBuilders.DoubleBuilder;

/**
 * Base subsystem for photon vision.
 */
public class PhotonVisionSubsystem extends SubsystemBase {
  private PhotonCamera m_limeLight;
  private PhotonPoseEstimator m_photonPoseEstimator;
  private Optional<EstimatedRobotPose> m_photonPose;
  private Transform3d m_target;
  private double targetX;
  private DoubleBuilder targetY;

  /**
   * Takes in the parmeters needed for photon vision.
   *
   */
  public PhotonVisionSubsystem() {
    PortForwarder.add(5800, "photonvision.local", 5800);

    m_limeLight = new PhotonCamera("OV5647");

    m_limeLight.setPipelineIndex(0);
    // 0: April Tags
    // 1: Reflective Tape

    m_photonPoseEstimator = new PhotonPoseEstimator(RobotConfig.VisionConfig.fieldLayout, 
    PoseStrategy.MULTI_TAG_PNP, m_limeLight,
        new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));
    m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    
    if (DriverStation.getAlliance() == Alliance.Red) {
      RobotConfig.VisionConfig.fieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
    } else {
      RobotConfig.VisionConfig.fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }
  }

  /**
   * Switches the pipeline of the Photonvision between april tag
   * and retroflective tape.
   */
  public void switchPipeLine() {
    if (m_limeLight.getPipelineIndex() == 0) {
      m_limeLight.setPipelineIndex(1);
    } else {
      m_limeLight.setPipelineIndex(0);
    }
  }

  public Optional<EstimatedRobotPose> getPhotonPose() {
    return m_photonPoseEstimator.update();
  }

  public PhotonPoseEstimator getPhotonPoseEstimator() {
    return m_photonPoseEstimator;
  }

  public Transform3d getTarget() {
    return m_target; 
  }

  /**
   * Returns if photon vision sees target
   * april tags or reflective tape depending on the set pipline. 
   *
   * @return 
   * 
   */
  public boolean hasTarget() {
    if (m_limeLight.getLatestResult().hasTargets()) {
      return true;
    } else {
      return false;
    }
  }

  public Pose2d getLimelightBasedPose() {
    if(hasTarget()){
      return new Pose2d(new Translation2d(m_limeLight.getLatestResult().getBestTarget().getBestCameraToTarget().getX(),m_limeLight.getLatestResult().getBestTarget().getBestCameraToTarget().getY())
      , m_limeLight.getLatestResult().getBestTarget().getBestCameraToTarget().getRotation().toRotation2d());
    }
    else {
      return new Pose2d();
    }
  }

  public Pose2d getFieldLimelightBasedPose() {
    int AprilTagID = 
      m_limeLight.getLatestResult().getBestTarget().getFiducialId();

    double xDistancefromRobottoAprilTag = 
      m_limeLight.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
    double yDistancefromRobottoAprilTag = 
      m_limeLight.getLatestResult().getBestTarget().getBestCameraToTarget().getY();

    double xPositionofAprilTag =  
      RobotConfig.VisionConfig.fieldLayout.getTagPose(
        m_limeLight.getLatestResult().getBestTarget().getFiducialId()).get().getX();
    double yPositionofAprilTag = 
      RobotConfig.VisionConfig.fieldLayout.getTagPose(
        m_limeLight.getLatestResult().getBestTarget().getFiducialId()).get().getY();
    
    Rotation2d rotationalComponent = 
    m_limeLight.getLatestResult().getBestTarget().getBestCameraToTarget().getRotation().toRotation2d();


    if(hasTarget()){
      if (DriverStation.getAlliance() == Alliance.Red) {
        if (AprilTagID < 5) {
          return new Pose2d(
            new Translation2d(
            xDistancefromRobottoAprilTag
            + 
            xPositionofAprilTag
            ,
            yDistancefromRobottoAprilTag
            +
            yPositionofAprilTag),
            rotationalComponent);
        } else {
          return new Pose2d(
            new Translation2d(
            xPositionofAprilTag
            - 
            xDistancefromRobottoAprilTag
            ,
            yPositionofAprilTag
            -
            yDistancefromRobottoAprilTag),
            rotationalComponent);
        }
      } else {
        if (AprilTagID > 4) {
          return new Pose2d(
            new Translation2d(
            xDistancefromRobottoAprilTag
            + 
            xPositionofAprilTag
            ,
            yDistancefromRobottoAprilTag
            +
            yPositionofAprilTag),
            rotationalComponent);
        } else {
          return new Pose2d(
            new Translation2d(
            xPositionofAprilTag
            - 
            xDistancefromRobottoAprilTag
            ,
            yPositionofAprilTag
            -
            yDistancefromRobottoAprilTag),
            rotationalComponent);
        }
      }
    }
    return new Pose2d();
  }
}
