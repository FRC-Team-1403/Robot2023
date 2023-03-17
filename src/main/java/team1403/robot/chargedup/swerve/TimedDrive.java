package team1403.robot.chargedup.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Drives the robot at the given chassis speeds for the given length in seconds. 
 */
public class TimedDrive extends CommandBase{
  private double m_startTime;
  private final double m_duration;
  private final ChassisSpeeds m_chassisSpeeds;
  private final SwerveSubsystem m_swerve;

  public TimedDrive(SwerveSubsystem swerve, double duration, ChassisSpeeds chassisSpeeds) {
    this.m_duration = duration;
    this.m_chassisSpeeds = chassisSpeeds;
    this.m_swerve = swerve;
  }

  @Override
  public void initialize() {
    this.m_startTime = Timer.getFPGATimestamp();
    System.out.println("Starting: ********************************************");
  }

  
  @Override
  public void execute() {
    m_swerve.drive(m_chassisSpeeds, new Translation2d());
  }

  @Override
  public void end(boolean interrupted) {    
    System.out.println("HVBfiuasfbhoun--------------------------------------------------------------------");
    m_swerve.stop();
    super.end(interrupted);
  }


  @Override
  public boolean isFinished() {
    System.out.println(Timer.getFPGATimestamp() - m_startTime);
    return Timer.getFPGATimestamp() - m_startTime >= m_duration;
  }

}
