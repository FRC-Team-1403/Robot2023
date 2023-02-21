// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot.chargedup.photonvision;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.robot.chargedup.RobotConfig.VisionConfig;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

public class PhotonVisionSubsystem extends CougarSubsystem {
  private PhotonCamera limeLight;
  private PhotonPoseEstimator photonPoseEstimator;
  private double targetYaw;
  private double targetPitch;
  private PIDController xController;
  private PIDController yController;
  private SwerveSubsystem m_drivetrain;
  private boolean reachedTarget = false;

  public PhotonVisionSubsystem(CougarLibInjectedParameters injectedParameters) {
    super("Vision Subsystem", injectedParameters);
    PortForwarder.add(5800, "photonvision.local", 5800);

    limeLight = new PhotonCamera("OV5647");

    limeLight.setPipelineIndex(1);
    // 0: April Tags
    // 1: Reflective Tape

    photonPoseEstimator = new PhotonPoseEstimator(VisionConfig.fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, limeLight,
        new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));
  }

  public void SwitchPipeline() {
    System.out.println("asdf");
    if (limeLight.getPipelineIndex() == 0) {
      limeLight.setPipelineIndex(1);
    } else {
      limeLight.setPipelineIndex(0);
    }
  }

  public void moveToTape(double pitch, double yaw, SwerveSubsystem drivetrain, List<PhotonTrackedTarget> targets) {
    for (int i = 0; i < targets.size(); i++) {
      if (targets.get(i).getYaw() > targets.get(0).getYaw()) {
        targetYaw = targets.get(i).getYaw();
        targetPitch = targets.get(i).getPitch();
      }
    }

    if (reachedTarget) {
      drivetrain.drive(
          new ChassisSpeeds(xController.calculate(yaw, targetYaw), yController.calculate(pitch, targetPitch), 0));
    }
    if (xController.calculate(yaw, targetYaw) <= 0.1 && yController.calculate(pitch, targetPitch) <= 0.1) {
      reachedTarget = true;
    }
  }

  public void moveToTag(SwerveSubsystem drivetrain, Pose2d targetPos) {
    if (reachedTarget) {
      drivetrain.drive(new ChassisSpeeds(xController.calculate(drivetrain.getPose().getX(), targetPos.getX()),
          yController.calculate(drivetrain.getPose().getY(), targetPos.getY()), 0));
    }
    if (xController.calculate(drivetrain.getPose().getX(), targetPos.getX()) <= 0.1
        && yController.calculate(drivetrain.getPose().getY(), targetPos.getY()) <= 0.1) {
      reachedTarget = true;
    }
  }

  public void updatePos(Pose2d pose) {
    m_drivetrain.setPose(new Pose2d(
        new Translation2d((m_drivetrain.getPose().getX() + pose.getX()) / 2,
            (m_drivetrain.getPose().getX() + pose.getY()) / 2),
        new Rotation2d(
            (m_drivetrain.getPose().getRotation().getRotations() + pose.getRotation().getRotations()) / 2)));
  }

  public Pose2d makePose2d(Pose3d pose) {
    return new Pose2d(new Translation2d(pose.getX(), pose.getY()),
        new Rotation2d(pose.getRotation().getX(), pose.getRotation().getY()));
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> photonPose = photonPoseEstimator.update();
    if (photonPose.isPresent()) {
      SmartDashboard.putString("Odometry", photonPose.get().estimatedPose.toString());
      moveToTag(m_drivetrain, makePose2d(photonPose.get().estimatedPose));
    }

    if (limeLight.getPipelineIndex() == 1) {
      // moveToTape(i, i,m_drivetrain, limeLight.getLatestResult().getTargets());
    }
  }
}
