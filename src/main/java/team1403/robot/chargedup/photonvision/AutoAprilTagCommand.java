package team1403.robot.chargedup.photonvision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import team1403.robot.chargedup.swerve.SwerveDrivePath;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

/**
 * Automatically aligns the robot to the nearest April Tag in the limelight's field of view.
 */
@SuppressWarnings("checkstyle:LocalVariableNameCheck")
public class AutoAprilTagCommand extends CommandBase {

  private final SwerveSubsystem m_drivetrainSubsystem;
  private final PhotonVisionSubsystem m_photonVisionSubsystem;

  private Pose2d m_lockedOnTarget;
  private SwerveDrivePath m_drivePathCommand;

  /**
   * Constructor for the April Tag command.
   *
   * @param drivetrainSubsystem The drivetrain subsystem used by the command.
   * @param photonVisionSubsystem The photonVision subsystem used by the command.
   */
  public AutoAprilTagCommand(SwerveSubsystem drivetrainSubsystem, 
                            PhotonVisionSubsystem photonVisionSubsystem) {

    m_drivetrainSubsystem = drivetrainSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;
  }

  @Override
  public void initialize() {
    double xPoseOfTarget = m_photonVisionSubsystem.getTarget().getX();
    double yPoseOfTarget = m_photonVisionSubsystem.getTarget().getY();

    double swerveSubsystemRotation = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees();
    double thetaOfTarget;

    //Find the rotation needed to align to the april tag
    if ((-90 < swerveSubsystemRotation) && (swerveSubsystemRotation < 90)) {
      thetaOfTarget = 1;
    } else {
      thetaOfTarget = 179;
    }

    m_lockedOnTarget = new Pose2d(new Translation2d(xPoseOfTarget, yPoseOfTarget), 
                                  new Rotation2d(thetaOfTarget));


    m_drivePathCommand = new SwerveDrivePath(
        m_drivetrainSubsystem,
        m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(),
        m_lockedOnTarget.getRotation().getDegrees(), 
        List.of(
          m_drivetrainSubsystem.getPose().getTranslation(),
          m_lockedOnTarget.getTranslation()));

    m_drivePathCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return m_drivePathCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivePathCommand.end(interrupted);
  }
}