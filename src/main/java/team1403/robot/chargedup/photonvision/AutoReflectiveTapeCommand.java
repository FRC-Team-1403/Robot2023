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

public class AutoReflectiveTapeCommand extends CommandBase {

  private SwerveSubsystem m_drivetrain;
  private PhotonVisionSubsystem m_vision;
  private double lowestX;
  private double lowestY;
  private SwerveDrivePath m_drivePathCommand;
  private Translation2d target;

  /** Creates a new AutoReflectiveTapeCommand. */
  public AutoReflectiveTapeCommand(SwerveSubsystem drivetrain, PhotonVisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_vision = vision;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double swerveSubsystemRotation = m_drivetrain.getGyroscopeRotation().getDegrees();
    double thetaOfTarget;
    lowestX = VisionConfig.reflectiveTapeLayout[0].getX();
    lowestY = VisionConfig.reflectiveTapeLayout[0].getY();
    for (int i = 1; i < 6; i++) {
      if (lowestX > VisionConfig.reflectiveTapeLayout[i].getX()
          && lowestY > VisionConfig.reflectiveTapeLayout[i].getY()) {
        lowestX = VisionConfig.reflectiveTapeLayout[i].getX();
        lowestY = VisionConfig.reflectiveTapeLayout[i].getY();
      }
    }
    if ((-3.14 / 2.0 < swerveSubsystemRotation) && (swerveSubsystemRotation < 3.14 / 2.0)) {
      thetaOfTarget = 1;
    } else {
      thetaOfTarget = 179;
    }
    target = new Translation2d(lowestX, lowestY);
    m_drivePathCommand = new SwerveDrivePath(
        m_drivetrain,
        m_drivetrain.getGyroscopeRotation().getDegrees(),
        thetaOfTarget,
        List.of(
            m_drivetrain.getOdometryValue().getTranslation(),
            target));

    m_drivePathCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivePathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivePathCommand.isFinished();
  }
}
