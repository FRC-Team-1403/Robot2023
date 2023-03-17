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

    // private double previousRollValue;
    // private double rollValue;
    // private double rollSetpoint;

    private double previousPitchValue;
    private double pitchValue;
    private double pitchMeasurement;

    // Difference between pitch 0.2 seconds ago compared to current pitch value
    private double pitchDifference;

    public SwerveAutoBalanceYaw(SwerveSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_xPIDController = new ProfiledPIDController(
            0.5, 
            RobotConfig.Swerve.kITranslation, 
            0,
            new TrapezoidProfile.Constraints(200, 200));
        m_drivetrainSubsystem.resetOdometry();
        previousPitchValue = 0;
        pitchValue = 0;
        pitchMeasurement = 0;
    }

    @Override
    public void execute() {
      pitchDifference = (previousPitchValue - m_drivetrainSubsystem.getGyroPitch()) * 20;
      pitchValue = m_drivetrainSubsystem.getGyroPitch() * .5;

      pitchMeasurement = pitchDifference + pitchValue;

      SmartDashboard.putNumber("Pitch value", pitchValue);
      System.out.println(pitchMeasurement);

      velocity = m_xPIDController.calculate(pitchMeasurement, 0);

      m_drivetrainSubsystem.drive(new ChassisSpeeds(velocity, 0, 0),
          new Translation2d());
  
      previousPitchValue = m_drivetrainSubsystem.getGyroPitch();
    }

    @Override
    public boolean isFinished() {
        if ((pitchDifference > -0.0001) && (pitchDifference < 0.0001)) {
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
        m_drivetrainSubsystem.setXModeEnabled(true);
    }
}
