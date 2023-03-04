package team1403.robot.chargedup.photonvision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
  private Matrix<N3,N1> stdMatrix;

  public PhotonVisionDefault(SwerveSubsystem drivetrainSubsystem,
      PhotonVisionSubsystem photonVisionSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;

    swervePoseEstimator = m_drivetrainSubsystem.getOdometer();

    photonPoseEstimator = m_photonVisionSubsystem.getPhotonPoseEstimator();
    stdMatrix = new Matrix<N3,N1>(Nat.N3(), Nat.N1());
    stdMatrix.fill(8.2);
// 8.2 
    m_drivetrainSubsystem.getOdometer().setVisionMeasurementStdDevs(stdMatrix);

    addRequirements(m_photonVisionSubsystem);
  }

  @Override
  public void initialize() {
    photonPoseEstimator.setReferencePose(m_drivetrainSubsystem.getOdometryValue());
    Optional<EstimatedRobotPose> result = m_photonVisionSubsystem.getPhotonPose();
    if (result.isPresent()) {
      EstimatedRobotPose photonPose = result.get();
      stdMatrix.fill(0.00001);
      swervePoseEstimator.addVisionMeasurement(
        photonPose.estimatedPose.toPose2d(), 
        photonPose.timestampSeconds, 
        stdMatrix);
      stdMatrix.fill(20);
      SmartDashboard.putString("Vision Odometry", photonPose.estimatedPose.toPose2d().toString());
    }
    
  }

  @Override
  public void execute() {
    Optional<EstimatedRobotPose> result = m_photonVisionSubsystem.getPhotonPose();
    if (result.isPresent()) {
      EstimatedRobotPose photonPose = result.get();
      if ((Math.abs(photonPose.estimatedPose.getX() - m_drivetrainSubsystem.getOdometryValue().getX()) < 0.8)
      || (Math.abs(photonPose.estimatedPose.getY() - m_drivetrainSubsystem.getOdometryValue().getY()) < 0.8)) {
      swervePoseEstimator.addVisionMeasurement(
          photonPose.estimatedPose.toPose2d(), photonPose.timestampSeconds);
      }
      SmartDashboard.putString("Vision Odometry", photonPose.estimatedPose.toPose2d().toString());
      SmartDashboard.putString("Vision X", photonPose.targetsUsed.toString());

      photonPoseEstimator.setReferencePose(m_drivetrainSubsystem.getOdometryValue());
    }
  }
}
