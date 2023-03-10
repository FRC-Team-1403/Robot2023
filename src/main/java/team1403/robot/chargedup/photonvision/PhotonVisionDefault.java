package team1403.robot.chargedup.photonvision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
  private Matrix<N3, N1> m_stdMatrix;

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
    m_stdMatrix = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
    m_stdMatrix.fill(8.2);
    m_drivetrainSubsystem.getOdometer().setVisionMeasurementStdDevs(m_stdMatrix);

    addRequirements(m_photonVisionSubsystem);
  }

  @Override
  public void initialize() {
    m_photonPoseEstimator.setReferencePose(m_drivetrainSubsystem.getOdometryValue());
    addVisionMeasurment();
  }

  @Override
  public void execute() {
    addVisionMeasurment();
  }

  /**
   * Adds a vision measurment to the drivetrain odometry.
   *
   * @return true if the vision measurment was added.
   */
  private boolean addVisionMeasurment() {
    Optional<EstimatedRobotPose> result = m_photonVisionSubsystem.getPhotonPose();
    if (result.isPresent()) {
      EstimatedRobotPose photonPose = result.get();
      double distanceFromCurrentMeasurment = photonPose.estimatedPose.toPose2d().getTranslation()
          .getDistance(m_drivetrainSubsystem.getOdometryValue().getTranslation());
      if (Math.abs(distanceFromCurrentMeasurment) <= 1) {
        m_swervePoseEstimator.addVisionMeasurement(
            photonPose.estimatedPose.toPose2d(),
            photonPose.timestampSeconds,
            m_stdMatrix);
        m_photonPoseEstimator.setReferencePose(m_drivetrainSubsystem.getOdometryValue());

        SmartDashboard.putString("Vision Odometry", photonPose.estimatedPose.toPose2d().toString());
        SmartDashboard.putString("Vision X", photonPose.targetsUsed.toString());

        return true;
      }
    }
    return false;
  }
}
