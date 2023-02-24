package team1403.robot.chargedup.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.chargedup.RobotConfig;

public class SwerveAutoBalanceYaw extends CommandBase {
 
    private final SwerveSubsystem m_drivetrainSubsystem;
    private final ProfiledPIDController m_xPIDController;
    private double velocity;
    private double previousRollValue;
    private double rollValue;
    private double rollSetpoint;

    // Roll velocity is in degrees per 20 milliseconds
    private double rollVelocity;

    public SwerveAutoBalanceYaw(SwerveSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_xPIDController = new ProfiledPIDController(
            0.5, 
            RobotConfig.SwerveConfig.kITranslation, 
            0,
            new TrapezoidProfile.Constraints(200, 200));
        m_drivetrainSubsystem.resetOdometry();
        previousRollValue = 0;
        rollValue = 0;
        rollSetpoint = 0;
    }

    @Override
    public void execute() {
      rollVelocity = ((previousRollValue - m_drivetrainSubsystem.getGyroRoll()) / 0.02) * -50;
      rollValue = m_drivetrainSubsystem.getGyroRoll() * .5;

      rollSetpoint = rollVelocity + rollValue;

      SmartDashboard.putNumber("roll velocity", rollSetpoint);
      System.out.println(rollSetpoint);

      velocity = m_xPIDController.calculate(rollSetpoint, 0);

      m_drivetrainSubsystem.drive(new ChassisSpeeds(velocity, 0, 0),
          new Translation2d());
  
      previousRollValue = m_drivetrainSubsystem.getGyroRoll();
    }

    @Override
    public boolean isFinished() {
        if ((rollVelocity > -0.0001) && (rollVelocity < 0.0001)) {
            return true;
        }
        // if ((m_drivetrainSubsystem.getGyroRoll() < 3) && (m_drivetrainSubsystem.getGyroRoll() > 0)) {
        //     return true;
        // }
        // if (rollVelocity != 0) {
        //     return true;
        // }
        // return false;
        return false;
    }

    @Override
    public void end(boolean interuptted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0.01), new Translation2d());
    }
}
