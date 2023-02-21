package team1403.robot.chargedup.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.chargedup.RobotConfig;
import team1403.robot.chargedup.RobotConfig.SwerveConfig;

/**
 * The default command for the swerve drivetrain subsystem.
 */
public class SwerveCommand extends CommandBase {
  private final SwerveSubsystem m_drivetrainSubsystem;

  private final DoubleSupplier m_verticalTranslationSupplier;
  private final DoubleSupplier m_horizontalTranslationSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final BooleanSupplier m_fieldRelativeSupplier;
  private boolean m_isFieldRelative;

  private final DoubleSupplier m_rightPivotSupplier;
  private final DoubleSupplier m_leftPivotSupplier;


  /**
   * Creates the swerve command.
   *\
   * @param drivetrain the instance of the {@link SwerveSubsystem}
   * @param horizontalTranslationSupplier
    supplies the horizontal speed of the drivetrain
   * @param verticalTranslationSupplier
    supplies the the vertical speed of the drivetrain
   * @param rotationSupplier supplies the rotational speed of the drivetrain
   * @param fieldRelativeSupplier supplies the
    boolean value to enable field relative mode
   */
  public SwerveCommand(SwerveSubsystem drivetrain,
                               DoubleSupplier horizontalTranslationSupplier,
                               DoubleSupplier verticalTranslationSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier fieldRelativeSupplier, 
                               DoubleSupplier rightPivotSupplier,
                               DoubleSupplier leftPivotSupplier) {
    this.m_drivetrainSubsystem = drivetrain;
    this.m_verticalTranslationSupplier = verticalTranslationSupplier;
    this.m_horizontalTranslationSupplier = horizontalTranslationSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_fieldRelativeSupplier = fieldRelativeSupplier;

    this.m_rightPivotSupplier = rightPivotSupplier;
    this.m_leftPivotSupplier = leftPivotSupplier;
    m_isFieldRelative = true;
    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void execute() {
    if (m_fieldRelativeSupplier.getAsBoolean()) {
      m_isFieldRelative = !m_isFieldRelative;
    }
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    double vertical = squareNum(m_verticalTranslationSupplier.getAsDouble()) * SwerveConfig.kMaxSpeed;
    double horizontal = squareNum(m_horizontalTranslationSupplier.getAsDouble()) * SwerveConfig.kMaxSpeed;
    double angular = squareNum(m_rotationSupplier.getAsDouble()) * SwerveConfig.kMaxAngularSpeed;
    Translation2d offset = new Translation2d();

    if(m_isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vertical, horizontal,
              angular, m_drivetrainSubsystem.getGyroscopeRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(vertical, horizontal, angular);
    }

    if (m_rightPivotSupplier.getAsDouble() > 0.08) {
      if (vertical >= 0.0) {
        // Pivots around front right wheel
        offset = new Translation2d(
          RobotConfig.SwerveConfig.kTrackWidth / 2.0, 
          -RobotConfig.SwerveConfig.kWheelBase / 2.0);
      } else {
        // Pivots around back right wheel
        offset = new Translation2d(
          -RobotConfig.SwerveConfig.kTrackWidth / 2.0,
          -RobotConfig.SwerveConfig.kWheelBase / 2.0);
      }
    }

    if (m_leftPivotSupplier.getAsDouble() > 0.08) {
      if (vertical >= 0.0) {
        // Pivots around front left wheel
        offset = new Translation2d(
          RobotConfig.SwerveConfig.kTrackWidth / 2.0,
          RobotConfig.SwerveConfig.kWheelBase / 2.0);
      } else {
        // Pivots around back left wheel
        offset = new Translation2d(
          -RobotConfig.SwerveConfig.kTrackWidth / 2.0,
          RobotConfig.SwerveConfig.kWheelBase / 2.0);
      }
    }
    m_drivetrainSubsystem.drive(chassisSpeeds, offset);
    SmartDashboard.putBoolean("isFieldRelative", m_isFieldRelative);
  }

  public double squareNum(double num) {
    double sign = Math.signum(num);
    return sign * Math.pow(num, 2);
  }
}
