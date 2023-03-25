package team1403.robot.chargedup;

import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.robot.chargedup.StateManager.GamePiece;
import team1403.robot.chargedup.arm.ArmStateGroup;
import team1403.robot.chargedup.arm.ArmSubsystem;
import team1403.robot.chargedup.arm.RunIntake;
import team1403.robot.chargedup.arm.SequentialMoveArmCommand;
import team1403.robot.chargedup.arm.SetpointArmCommand;
import team1403.robot.chargedup.swerve.SwerveControllerCommand;
import team1403.robot.chargedup.swerve.SwerveSubsystem;
import team1403.robot.chargedup.swerve.TimedDrive;

public class AutoManager {
  static private AutoManager m_instance;
  static private final TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(6,
      RobotConfig.Swerve.kMaxAccelerationMetersPerSecondSquared).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private AutoManager() {}


  public static AutoManager getInstance() {
    if(m_instance == null) {
      m_instance = new AutoManager();
    }
    return m_instance;
  }
  /**
   * Command for grids 1 and 3.
   * @param swerve
   * @param arm
   * @return the command
   */
  Command getRightGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    //Config navx
    // swerve.zeroGyroscope();

    swerve.setSpeedLimiter(1);

    // swerve.setYawGyroscopeOffset(180);
    Rotation2d rotation = swerve.getGyroscopeRotation();

    // 2. Generate trajectory
    // m_trajectoryConfig.setReversed(true);
    final Trajectory trajectory0 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(
            new Translation2d(-0.2, 0),
            new Translation2d(-0.5, 0)
            ),
        new Pose2d(-0.5, 0, Rotation2d.fromDegrees(0)),
        m_trajectoryConfig);

    final Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-0.5, 0, Rotation2d.fromDegrees(0)),
        List.of(
            new Translation2d(-1, 0),
            new Translation2d(-2.5, 0.1)
            ),
        new Pose2d(-5, 0, Rotation2d.fromDegrees(179)),
        m_trajectoryConfig);

      final Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-5, 0, Rotation2d.fromDegrees(179)),
        List.of(
            new Translation2d(-2.5, 0.1),
            new Translation2d(-1, 0)
            ),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        m_trajectoryConfig);

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
      5.3,
      RobotConfig.Swerve.kIAutoTurning,
      RobotConfig.Swerve.kDAutoTurning,
      RobotConfig.Swerve.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommandZero = new SwerveControllerCommand(
        trajectory0,
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);
    
    SwerveControllerCommand swerveControllerCommandOne = new SwerveControllerCommand(
        trajectory1,
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

      SwerveControllerCommand swerveControllerCommandTwo = new SwerveControllerCommand(
        trajectory2,
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    return new SequentialCommandGroup(
      new SequentialMoveArmCommand(arm, StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
      new RunIntake(arm, 1),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.1),
          new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
          new WaitCommand(0.8),
          new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
          new SetpointArmCommand(arm, () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(), true),
          new RunIntake(arm, -1),
          new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true)
        ),
        new SequentialCommandGroup(
          swerveControllerCommandZero,
          swerveControllerCommandOne
        )
      ),
      swerveControllerCommandTwo
      );
  }


  public Command getTimedSideGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
      new SequentialMoveArmCommand(arm, StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
      new RunIntake(arm, 1),
      new TimedDrive(swerve, 1, new ChassisSpeeds(-2, 0, 0)),
      new ParallelCommandGroup(
        new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
        new TimedDrive(swerve,6.5, new ChassisSpeeds(-2, 0, 0))
      )
      );
  }

  public Command getMiddleGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
  swerve.setSpeedLimiter(1);
  return new SequentialCommandGroup(
      new TimedDrive(swerve, 3.4, new ChassisSpeeds(-4, 0, 0)),
      new TimedDrive(swerve, 1.6, new ChassisSpeeds(0, -4, 0)),
      new TimedDrive(swerve, 1.8, new ChassisSpeeds(4, 0, 0)),
      new InstantCommand(() -> swerve.setXModeEnabled(true))
  );
  }
}