// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot.chargedup.photonvision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.robot.chargedup.StateManager;
import team1403.robot.chargedup.RobotConfig.Vision;
import team1403.robot.chargedup.StateManager.GamePiece;

public class PhotonVisionSubsystem extends CougarSubsystem {
  private PhotonCamera limeLight;
  private PhotonPoseEstimator photonPoseEstimator;

  private final IntegerSubscriber m_coneOrientationSubsriber;

  public PhotonVisionSubsystem(CougarLibInjectedParameters injectedParameters) {
    super("Vision Subsystem", injectedParameters);
    // Photonvision
    PortForwarder.add(5800, "photonvision.local", 5800);
    limeLight = new PhotonCamera("OV5647");
    // 0: April Tags
    // 1: Reflective Tape
    limeLight.setPipelineIndex(0);
    photonPoseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, limeLight,
        new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));

    //Cone detection
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable coneTable = instance.getTable("conetable");
    m_coneOrientationSubsriber = coneTable.getIntegerTopic("rotcone").subscribe(-1);
  }

  @Override
  public void periodic() {
    int coneOrientation = (int) m_coneOrientationSubsriber.get();
    StateManager.getInstance().updateArmState(GamePiece.fromInt(coneOrientation));
    SmartDashboard.putString("Game Piece", StateManager.getInstance().getGamePiece().name());
    super.periodic();
  }
}
