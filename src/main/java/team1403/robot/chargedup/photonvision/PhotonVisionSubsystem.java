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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.robot.chargedup.RobotConfig.VisionConfig;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

/**
 * creates the photon vision class.
 */
public class PhotonVisionSubsystem extends CougarSubsystem {
  private PhotonCamera limeLight;
  private PhotonPoseEstimator photonPoseEstimator;

  /**
   * Initializes the photon vision subsystem. 
   *
   * @param injectedParameters the cougar injected parameters
   */
  public PhotonVisionSubsystem(CougarLibInjectedParameters injectedParameters) {
    super("Vision Subsystem", injectedParameters);
    // m_drivetrain = drivetrain;
    PortForwarder.add(5800, "photonvision.local", 5800);

    limeLight = new PhotonCamera("OV5647");

    limeLight.setPipelineIndex(0);
    // 0: April Tags
    // 1: Reflective Tape

    photonPoseEstimator = new PhotonPoseEstimator(VisionConfig.fieldLayout,
    PoseStrategy.LOWEST_AMBIGUITY, limeLight,
        new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));
  }

  public void SwitchPipeline() {
    if (limeLight.getPipelineIndex() == 0) {
      limeLight.setPipelineIndex(1);
    } else {
      limeLight.setPipelineIndex(0);
    }
  }

  public void moveToTape(double pitch, double yaw, SwerveSubsystem drivetrain) {
  }

  public void updatePos(Pose2d pose) {
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> photonPose = photonPoseEstimator.update();
    if (photonPose.isPresent()) {
      SmartDashboard.putString("Odometry", photonPose.get().estimatedPose.toString());
    }

    var targets = limeLight.getLatestResult().getTargets();
    if (limeLight.getLatestResult().hasTargets()) {
    }
    int lowestTargetIndex = 0;
    for (int i = 0; i < targets.size(); i++) {
      if (targets.get(i).getPitch() < targets.get(lowestTargetIndex).getPitch()) {
        lowestTargetIndex = i;
      }

      SmartDashboard.putNumber("Reflective Tape Distance", 
          PhotonUtils.calculateDistanceToTargetMeters(0.727, 0.662,
          Math.PI / 60, targets.get(lowestTargetIndex).getPitch()));
    }
  }
}
