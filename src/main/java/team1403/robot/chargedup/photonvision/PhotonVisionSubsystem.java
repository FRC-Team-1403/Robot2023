// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot.chargedup.photonvision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import edu.wpi.first.wpilibj.XboxController;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
  private PhotonTrackedTarget target = new PhotonTrackedTarget();
  private double previousPipelineTimestamp =0;
  private Transform3d CAM_TO_ROBOT;
  private static NetworkTable table; 
  private Transform3d camToTarget;
  private Transform3d camToTargetTape;
  private double XDistance;
  private double YDistance;
  private double ZDistance;
  private ArrayList<AprilTag> tagList;
  private AprilTagFieldLayout fieldLayout;

  public PhotonVisionSubsystem() {


    table = NetworkTableInstance.getDefault().getTable("OV5647");

    PortForwarder.add(5810, "photonvision.local", 5810);
      
    
    camToTarget =  new Transform3d();

    final AprilTag tag01 = new AprilTag(1,new Pose3d(15.51,1.07,0.46,new Rotation3d(15.51,1.07,0.46)));

    final AprilTag tag02 = new AprilTag(2,new Pose3d(15.51,2.75,0.46,new Rotation3d(15.51,2.75,0.46)));

    final AprilTag tag03 = new AprilTag(3, new Pose3d(15.51,4.43,0.46, new Rotation3d(15.51,4.43,0.46)));

    final AprilTag tag04  = new AprilTag(4, new Pose3d(16.18,6.75,0.695, new Rotation3d(16.18,6.75,0.695))); 

    final AprilTag tag05 = new AprilTag(5, new Pose3d(0.36, 6.75,0.695, new Rotation3d(0.36, 6.75,0.695)));

    final AprilTag tag06 = new AprilTag(6,  new Pose3d(1.03, 4.43,0.46, new Rotation3d(1.03, 4.43,0.46))); 

    final AprilTag tag07 = new AprilTag(7,  new Pose3d(1.03, 2.75,0.46, new Rotation3d(1.03, 2.75,0.46))); 
    
    final AprilTag tag08 = new AprilTag(8, new Pose3d(1.03, 1.07, 0.46, new Rotation3d(1.03, 1.07, 0.46))); 

    tagList = new ArrayList<AprilTag>();
    tagList.add(tag01);
    tagList.add(tag02);
    tagList.add(tag03);
    tagList.add(tag04);
    tagList.add(tag05);
    tagList.add(tag06);
    tagList.add(tag07);
    tagList.add(tag08);
    fieldLayout = new AprilTagFieldLayout(tagList,16.54  ,8.02 );
    photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, limeLight, CAM_TO_ROBOT);

      limeLight =  new PhotonCamera("OV5647");
      
     // pipelineEntry = table.getEntry("pipeline");
      limeLight.setPipelineIndex(0);
      // 0: April Tags
      // 1: Reflective Tape
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

  public void SwitchPipeline(){
    if(limeLight.getPipelineIndex() == 0){
      limeLight.setPipelineIndex(1);
    }else{
      limeLight.setPipelineIndex(0);
    }
  }

  @Override
  public void periodic() {

    var pipelineResult = limeLight.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if(resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()){
        previousPipelineTimestamp = resultTimestamp;
        var target = pipelineResult.getBestTarget();
        var fiducialId = target.getFiducialId();

        if(target.getPoseAmbiguity() <= 1 && fiducialId >= 0){
            camToTarget = target.getBestCameraToTarget();

        }
    
      }

      if(limeLight.getPipelineIndex() == 1){
          camToTargetTape = pipelineResult.getBestTarget().getBestCameraToTarget();
          SmartDashboard.putNumber("X distance for tape", camToTargetTape.getX());
          
      }
      XDistance = ((1.11*camToTarget.getX())-0.173);
      YDistance = ((-1.28*Math.pow((camToTarget.getY()), 2)) -(0.668 * camToTarget.getY())-0.3);
      ZDistance = (camToTarget.getZ()*1.1);

  

      SmartDashboard.putNumber("Tag Yaw", target.getYaw());
      SmartDashboard.putNumber("Tag Pitch", target.getPitch());
      SmartDashboard.putNumber("Tag Skew", target.getSkew());
      SmartDashboard.putNumber("Tag FiducialId", target.getFiducialId());
      SmartDashboard.putNumber("Tag Pose Ambiguity",target.getPoseAmbiguity());
      SmartDashboard.putNumber("Pipeline ID", limeLight.getPipelineIndex());      
      SmartDashboard.putNumber("X Distance",camToTarget.getX());
      SmartDashboard.putNumber("Y distance ", camToTarget.getY());
      SmartDashboard.putNumber("Z Distance", camToTarget.getZ());
      SmartDashboard.putNumber("x Distance with offset",XDistance);
      SmartDashboard.putNumber("Y Distance with offset", YDistance);
      SmartDashboard.putNumber("Z Distance with offset", ZDistance);

  }
}