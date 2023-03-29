// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot.chargedup.photonvision;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import team1403.robot.chargedup.RobotConfig.VisionConfig;
import team1403.robot.chargedup.swerve.SwerveDrivePath;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

/**
 * Automatically aligns the robot to the nearest refelctive tape grid 
 * scoring location using the drivetrain's odometry. 
 */
public class AutoReflectiveTapeCommand extends CommandBase {

  private SwerveSubsystem m_drivetrain;
  private double m_lowestX;
  private double m_lowestY;
  private SwerveDrivePath m_drivePathCommand;
  private Translation2d m_target;

  /** Creates a new AutoReflectiveTapeCommand. */
  public AutoReflectiveTapeCommand(SwerveSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Find the closest reflective tape
    m_lowestX = VisionConfig.reflectiveTapeLayout[0].getX();
    m_lowestY = VisionConfig.reflectiveTapeLayout[0].getY();
    for (int i = 1; i < 6; i++) {
      if (m_lowestX > VisionConfig.reflectiveTapeLayout[i].getX()
          && m_lowestY > VisionConfig.reflectiveTapeLayout[i].getY()) {
        m_lowestX = VisionConfig.reflectiveTapeLayout[i].getX();
        m_lowestY = VisionConfig.reflectiveTapeLayout[i].getY();
      }
    }
    
    //Find the rotation needed to align to the reflective tape
    double thetaOfTarget;
    double swerveSubsystemRotation = m_drivetrain.getGyroscopeRotation().getDegrees();
    if ((-90 < swerveSubsystemRotation) && (swerveSubsystemRotation < 90)) {
      thetaOfTarget = 1;
    } else {
      thetaOfTarget = 179;
    }

    //Call the command to align to the reflective tape
    m_target = new Translation2d(m_lowestX, m_lowestY);
    m_drivePathCommand = new SwerveDrivePath(
        m_drivetrain,
        m_drivetrain.getGyroscopeRotation().getDegrees(),
        thetaOfTarget,
        List.of(
            m_drivetrain.getPose().getTranslation(),
            m_target));

    m_drivePathCommand.schedule();
  }


  @Override
  public void end(boolean interrupted) {
    m_drivePathCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_drivePathCommand.isFinished();
  }
}
