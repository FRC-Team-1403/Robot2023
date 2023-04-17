package team1403.robot.chargedup.photonvision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

public class AutoAlignCommand extends CommandBase {
  private final SwerveSubsystem m_swerve;
  private final PhotonVisionSubsystem m_vision;

  private final PIDController m_yawController;
  private final PIDController m_distanceController;
  private final PIDController m_thetaController;

  private boolean isRotated = false;
  private boolean isTranslated = false;

  /**
   * Auto align the robot to be in the perfect place to score
   * 
   * @param swerve drivetrain subsystem
   * @param vision photonvision subsystem
   */
  public AutoAlignCommand(SwerveSubsystem swerve, PhotonVisionSubsystem vision) {
    m_swerve = swerve;
    m_vision = vision;

    m_yawController = new PIDController(12, 0, 0.5);
    m_distanceController = new PIDController(12, 0, 0.5);
    m_thetaController = new PIDController(4, 0, 0);
  }

  @Override
  public void execute() {
    rotateToTarget();
    translateToTarget();
  }

  @Override
  public boolean isFinished() {
    // If robot is within acceptable bounds for position & has been sucessfully rotated
    return isRotated && isTranslated;
  }

  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(new ChassisSpeeds(), new Translation2d());
  }

  /**
   * Give swerve chassis a rotational velocity
   * 
   * @return when swerve is rotated towards target
   */
  private void rotateToTarget() {
    double rotationOfSwerve = m_swerve.getGyroscopeRotation().getDegrees();

    // If rotation is within acceptable bounds
    if (Math.abs(rotationOfSwerve - 180) <= 0.2) {
      isRotated = true;
      return;
    } 

    double rotationalSpeed = m_thetaController.calculate(rotationOfSwerve, 180);
    m_swerve.drive(
      new ChassisSpeeds(0, 0, rotationalSpeed), 
      new Translation2d());
  }

  /**
   * Give swerve chassis a translation velocity
   * 
   * @return when swerve is aligned with target
   */
  private void translateToTarget() {
    double horizontalOffest = m_vision.getHorizontalOffsetOfTarget();
    double distanceOffset = m_vision.getDistanceFromTarget();

    if ((Math.abs(horizontalOffest) <= 0.2) && (Math.abs(distanceOffset - 1) <= 0.2)) {
      isTranslated = true;
      return;
    }

    double xVelocity = m_yawController.calculate(horizontalOffest, 0);
    double yVelocity = m_distanceController.calculate(distanceOffset, 1.5);

    m_swerve.drive(
      new ChassisSpeeds(xVelocity, yVelocity, 0), 
      new Translation2d());
  }
}
