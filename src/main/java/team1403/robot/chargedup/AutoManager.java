package team1403.robot.chargedup;

import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team1403.robot.chargedup.arm.ArmSubsystem;
import team1403.robot.chargedup.arm.RunIntake;
import team1403.robot.chargedup.swerve.SwerveControllerCommand;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

public class AutoManager {
  static private AutoManager m_instance;

  private AutoManager() {}


  public static AutoManager getInstance() {
    if(m_instance == null) {
      m_instance = new AutoManager();
    }
    return m_instance;
  }

  Command getSideGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    //Config navx
    swerve.zeroGyroscope();
    swerve.setGyroscopeOffset(199);

    Rotation2d rotation = swerve.getGyroscopeRotation();

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      RobotConfig.Swerve.kMaxSpeed,
      3)
      .setKinematics(RobotConfig.Swerve.kDriveKinematics);

    // 2. Generate trajectory
    final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, rotation),
        List.of(
            new Translation2d(0.2, 0),
            new Translation2d(0.4, 0),
            new Translation2d(0.6, 0),
            new Translation2d(0.8, 0),
            new Translation2d(0.9, 0)),
        new Pose2d(1, 0, rotation),
        trajectoryConfig);

    Trajectory trajectoryTwo = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1, 0, rotation),
      List.of(
          new Translation2d(1.5, 0),
          new Translation2d(2, 0),
          new Translation2d(2.5, 0),
          new Translation2d(3, 0)),
      new Pose2d(3.5, 0, rotation),
      trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(
      RobotConfig.Swerve.kPTranslation, 
      RobotConfig.Swerve.kITranslation, 
      RobotConfig.Swerve.kDTranslation);

    PIDController yController = new PIDController(
      RobotConfig.Swerve.kPTranslation, 
      RobotConfig.Swerve.kITranslation, 
      RobotConfig.Swerve.kDTranslation);

    ProfiledPIDController thetaController = new ProfiledPIDController(
      0, 
      RobotConfig.Swerve.kIAutoTurning, 
      RobotConfig.Swerve.kDAutoTurning, 
      RobotConfig.Swerve.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommandOne = new SwerveControllerCommand(
        trajectory,
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    SwerveControllerCommand swerveControllerCommandTwo = new SwerveControllerCommand(
      trajectoryTwo,
      swerve::getPose,
      xController,
      yController,
      thetaController,
      swerve);
  
    // return swerveControllerCommand;

    return new SequentialCommandGroup(
      // new SequentialMoveArmCommand(m_arm, StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
      new RunIntake(arm, 1),
      swerveControllerCommandOne,
      // new SetpointArmCommand(m_arm, ArmStateGroup.getTuck(), false),
      swerveControllerCommandTwo
      );
  }
}