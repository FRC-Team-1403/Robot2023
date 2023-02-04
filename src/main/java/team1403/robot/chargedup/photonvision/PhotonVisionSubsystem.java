// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot.chargedup.photonvision;

import java.util.Collections;
import java.util.List;
import java.util.Optional;
import edu.wpi.first.wpilibj.XboxController;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  
  private PhotonCamera limeLight;
  private PhotonPoseEstimator photonPoseEstimator;
  private PhotonPipelineResult result;
  private List<Pose3d> targetPoses;
  // private boolean hasTargets;
  // private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget target = new PhotonTrackedTarget();
  private double previousPipelineTimestamp =0;
  private Transform3d CAM_TO_ROBOT;
  private Shuffleboard shuffleboard;
  private NetworkTableEntry networkTableEntry;
  private static NetworkTable table; 
  private CameraServer cameraServer;
  private int AprilTagIndex;
  private int TapeIndex;

  private int indexNum = 1;

  private NetworkTableEntry pipelineEntry;
  private XboxController m_Controller;

  public PhotonVisionSubsystem() {

    // int currentIndex = limeLight.getPipelineIndex();

    table = NetworkTableInstance.getDefault().getTable("OV5647");
    m_Controller = new XboxController(0);

    PortForwarder.add(5810, "photonvision.local", 5810);
    
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
    
      limeLight =  new PhotonCamera("OV5647");
      // AprilTagIndex = limeLight.getPipelineIndex();
      
      pipelineEntry = table.getEntry("pipeline");
      limeLight.setPipelineIndex(1);
      // 1: April Tags
      // 0: Reflective Tape
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

    // Timer.delay(10);

    // if(m_Controller.getAButtonPressed()){
    //   limeLight.setPipelineIndex(1);
    // } else {
    //   limeLight.setPipelineIndex(0);
    // } 

    var pipelineResult = limeLight.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if(resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()){
        previousPipelineTimestamp = resultTimestamp;
        var target = pipelineResult.getBestTarget();
        var fiducialId = target.getFiducialId();

        if(target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId < targetPoses.size() ){
            var targetpose = targetPoses.get(fiducialId);
            Transform3d camToTarget = target.getBestCameraToTarget();

        }
      SmartDashboard.putNumber("Tag Yaw", target.getYaw());
      SmartDashboard.putNumber("Tag Pitch", target.getPitch());
      SmartDashboard.putNumber("Tag Skew", target.getSkew());
      SmartDashboard.putNumber("Tag FiducialId", target.getFiducialId());
      SmartDashboard.putNumber("Tag Pose Ambiguity",target.getPoseAmbiguity());
      SmartDashboard.putNumber("Tag ID", limeLight.getPipelineIndex());
      
    }
  }
  }
