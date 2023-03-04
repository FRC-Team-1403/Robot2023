// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot.chargedup.photonvision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.robot.chargedup.RobotConfig.VisionConfig;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * Base subsystem for photon vision.
 */
public class PhotonVisionSubsystem extends CougarSubsystem {
  private PhotonCamera m_limeLight;
  private PhotonPoseEstimator m_photonPoseEstimator;
  private Optional<EstimatedRobotPose> m_photonPose;
  private Transform3d m_target;

  /**
   * Takes in the parmeters needed for photon vision.
   *
   * @param injectedParameters
   * 
   */
  public PhotonVisionSubsystem(CougarLibInjectedParameters injectedParameters) {
    super("Vision Subsystem", injectedParameters);
    PortForwarder.add(5800, "photonvision.local", 5800);

    m_limeLight = new PhotonCamera("OV5647");

    m_limeLight.setPipelineIndex(0);
    // 0: April Tags
    // 1: Reflective Tape

    m_photonPoseEstimator = new PhotonPoseEstimator(VisionConfig.fieldLayout, 
    PoseStrategy.MULTI_TAG_PNP, m_limeLight,
        new Transform3d(new Translation3d(12, 8, 30), new Rotation3d(0.0523599, 0, 0)));

    m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
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
    return m_photonPose.get().estimatedPose.toPose2d();
  }
}
