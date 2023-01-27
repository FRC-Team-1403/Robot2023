// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot.chargedup.photonvision;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  private PhotonCamera limeLight;
  private PhotonPoseEstimator photonPoseEstimator;
  private SimVisionSystem simVision;
  private PhotonPipelineResult result;
  private HttpCamera m_camera;
  private List<Pose3d> targetPoses;
  // private boolean hasTargets;
  // private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget target = new PhotonTrackedTarget();
  private double previousPipelineTimestamp =0;
  private int arraySize;
  private ArrayList atList;
  private Transform3d CAM_TO_ROBOT;

  public PhotonVisionSubsystem() {
    //var cameraPose  = robotPose.transformBy(ROBOT_TO_CAMERA); 
  
    targetPoses = Collections.unmodifiableList(List.of(
        
    new Pose3d(610.77,42.19,18.22,new Rotation3d(610.77, 42.19, 18.22)),

    new Pose3d(610.77,108.19,18.22,new Rotation3d(610.77, 108.19, 18.22)),

    new Pose3d(610.77,174.19,18.22, new Rotation3d(610.77, 174.19, 18.22)),

    new Pose3d(610.77,174.19,18.22, new Rotation3d(610.77, 174.19, 18.22)),

    new Pose3d(636.96,265.74,27.38, new Rotation3d(636.96, 265.74, 27.38)),
    
    new Pose3d(14.25, 265.74,27.38, new Rotation3d(14.25, 265.74, 27.38)), 

    new Pose3d(40.45, 174.19,18.22, new Rotation3d(40.45, 174.19, 18.22)),
    
    new Pose3d(40.45, 108.19,18.22, new Rotation3d(40.45, 108.19, 18.22)),
    
    new Pose3d(40.45, 42.19, 18.22, new Rotation3d(40.45, 42.19, 18.22))
    
    ));
    

      limeLight = 
      new PhotonCamera("Limelight");
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator.update();
  }


  public List<PhotonTrackedTarget> getTargets(){
    return result.getTargets();
  }

  public PhotonTrackedTarget getBestTarget(){
    return result.getBestTarget();
  }


  @Override
  public void periodic() {
    var pipelineResult = limeLight.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if(resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()){
        previousPipelineTimestamp = resultTimestamp;
        var target = pipelineResult.getBestTarget();
        var fiducialId = target.getFiducialId();

        if(target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId < targetPoses.size() ){
            var targetpose = targetPoses.get(fiducialId);
            Transform3d camToTarget = target.getBestCameraToTarget();
            Pose3d camPose = targetpose.transformBy(camToTarget.inverse());

            var visionMeasurment = camPose.transformBy(CAM_TO_ROBOT);

            SmartDashboard.putNumber("Yaw", target.getYaw());
            SmartDashboard.putNumber("Pitch", target.getPitch());

            
        }
    }

    
  }
}
