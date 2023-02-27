package team1403.robot.chargedup.photonvision;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.chargedup.RobotConfig.SwerveConfig;
import team1403.robot.chargedup.swerve.SwerveControllerCommand;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

public class AutoAprilTagCommand extends CommandBase {
  
  private final SwerveSubsystem m_drivetrainSubsystem;
  private final PhotonVisionSubsystem m_photonVisionSubsystem;

  private Pose2d lockedOnTarget;
  
  private final Trajectory m_trajectory;
  private final TrajectoryConfig m_trajectoryConfig;

  private final SwerveControllerCommand m_swerveControllerCommand;

  private final PIDController m_verticalTranslationController;
  private final PIDController m_horizontalTranslationController;
  private final ProfiledPIDController m_angleController;

  public AutoAprilTagCommand(SwerveSubsystem drivetrainSubsystem, 
        PhotonVisionSubsystem photonVisionSubsystem) {

      m_drivetrainSubsystem = drivetrainSubsystem;
      m_photonVisionSubsystem = photonVisionSubsystem;

    m_trajectoryConfig = new TrajectoryConfig(
      SwerveConfig.kMaxSpeed / 4,
      SwerveConfig.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(SwerveConfig.kDriveKinematics);

    m_verticalTranslationController = new PIDController(SwerveConfig.kPTranslation, 
        SwerveConfig.kITranslation, SwerveConfig.kDTranslation);
    m_horizontalTranslationController = new PIDController(SwerveConfig.kPTranslation, 
        SwerveConfig.kITranslation, SwerveConfig.kDTranslation);

    m_angleController = new ProfiledPIDController(
        SwerveConfig.kPAutoTurning, SwerveConfig.kIAutoTurning, 
        SwerveConfig.kDAutoTurning, SwerveConfig.kThetaControllerConstraints);

    double xPoseOfTarget = m_photonVisionSubsystem.getTarget().getX();
    double yPoseOfTarget = m_photonVisionSubsystem.getTarget().getY();

    double swerveSubsystemRotation = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees();
    double thetaOfTarget;

    if  ((-3.14/2.0 < swerveSubsystemRotation) && (swerveSubsystemRotation < 3.14/2.0)) {
      thetaOfTarget = 1;
    } else {
      thetaOfTarget = 179;
    }

    try {
     lockedOnTarget = new Pose2d(new Translation2d(xPoseOfTarget,yPoseOfTarget), new Rotation2d(thetaOfTarget));
    } catch (NullPointerException e) {
      lockedOnTarget = new Pose2d();
    }

    m_trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        m_drivetrainSubsystem.getOdometryValue(),
        lockedOnTarget
      ), m_trajectoryConfig);

    m_swerveControllerCommand = new SwerveControllerCommand(
      m_trajectory,
      m_drivetrainSubsystem::getPose,
      m_horizontalTranslationController,
      m_verticalTranslationController,
      m_angleController,
      m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    m_drivetrainSubsystem.increaseSpeed(1);

    m_swerveControllerCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return m_swerveControllerCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.decreaseSpeed(0.4);
    m_drivetrainSubsystem.stop();
  }
}