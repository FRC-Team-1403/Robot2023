package team1403.robot.chargedup.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

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

  /**
   * Creates the swerve command.
   *
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
                               BooleanSupplier fieldRelativeSupplier) {
    this.m_drivetrainSubsystem = drivetrain;
    this.m_verticalTranslationSupplier = verticalTranslationSupplier;
    this.m_horizontalTranslationSupplier = horizontalTranslationSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_fieldRelativeSupplier = fieldRelativeSupplier;
    m_isFieldRelative = true;
    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void execute() {
    if (m_fieldRelativeSupplier.getAsBoolean()) {
      m_isFieldRelative = !m_isFieldRelative;
    }

    SmartDashboard.putBoolean("isFieldRelative", m_isFieldRelative);
    if (m_isFieldRelative) {
      m_drivetrainSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              m_verticalTranslationSupplier.getAsDouble() * SwerveConfig.kMaxSpeed,
              m_horizontalTranslationSupplier.getAsDouble() * SwerveConfig.kMaxSpeed,
              m_rotationSupplier.getAsDouble() * SwerveConfig.kMaxAngularSpeed,
              m_drivetrainSubsystem.getGyroscopeRotation()));
    } else {
      m_drivetrainSubsystem.drive(
          new ChassisSpeeds(
              m_verticalTranslationSupplier.getAsDouble() * SwerveConfig.kMaxSpeed,
              m_horizontalTranslationSupplier.getAsDouble() * SwerveConfig.kMaxSpeed,
              m_rotationSupplier.getAsDouble() * SwerveConfig.kMaxAngularSpeed));
    }
  }
}
