package team1403.robot.chargedup;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  private final TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(4,
      2).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final PIDController xController = new PIDController(
      RobotConfig.Swerve.kPTranslation,
      RobotConfig.Swerve.kITranslation,
      RobotConfig.Swerve.kDTranslation);

  private final PIDController yController = new PIDController(
      RobotConfig.Swerve.kPTranslation,
      RobotConfig.Swerve.kITranslation,
      RobotConfig.Swerve.kDTranslation);

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
      4,
      RobotConfig.Swerve.kIAutoTurning,
      RobotConfig.Swerve.kDAutoTurning,
      RobotConfig.Swerve.kThetaControllerConstraints);

  private SwerveControllerCommand grid3Trajectory1;
  private SwerveControllerCommand grid3Trajectory2;
  private SwerveControllerCommand grid3Trajectory3;

  private AutoManager() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public static AutoManager getInstance() {
    if (m_instance == null) {
      m_instance = new AutoManager();
    }
    return m_instance;
  }

  public void init(SwerveSubsystem swerve) {
    grid3Trajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, -0.3),
                new Translation2d(-2, -0.3)),
            new Pose2d(-3, 0, Rotation2d.fromDegrees(180)),
            m_trajectoryConfig),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    grid3Trajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(-3, 0, Rotation2d.fromDegrees(180)),
            List.of(
                new Translation2d(-4.8, 0.7),
                new Translation2d(-4.9, -0.7),
                new Translation2d(-4.7, -0.2)),
            new Pose2d(-4, 0.05, Rotation2d.fromDegrees(225)),
            m_trajectoryConfig),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    grid3Trajectory3 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-4, -0.3, Rotation2d.fromDegrees(225)),
            List.of(
                new Translation2d(-2.5, -0.5),
                new Translation2d(-0.5, -0.5)),
            new Pose2d(0.1, -1.05, Rotation2d.fromDegrees(4)),
            m_trajectoryConfig),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);
  }

  /**
   * Command for grids 3.
   * 
   * @param swerve
   * @param arm
   * @return the command
   */
  public Command getRightGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);

    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
        StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(1.8),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new SetpointArmCommand(arm, () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 3, 1.5),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new WaitCommand(1.2),
                new SetpointArmCommand(arm, () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false)
                ),
            new SequentialCommandGroup(
                grid3Trajectory1,
                grid3Trajectory2,
                grid3Trajectory3)),
                new RunIntake(arm, -1));
  }

  public Command getTimedSideGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm, StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
        new RunIntake(arm, 1),
        new TimedDrive(swerve, 1, new ChassisSpeeds(-2, 0, 0)),
        new ParallelCommandGroup(
            new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
            new TimedDrive(swerve, 6.5, new ChassisSpeeds(-2, 0, 0))));
  }

  public Command getMiddleGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new TimedDrive(swerve, 3.4, new ChassisSpeeds(-4, 0, 0)),
        new TimedDrive(swerve, 1.6, new ChassisSpeeds(0, -4, 0)),
        new TimedDrive(swerve, 1.8, new ChassisSpeeds(4, 0, 0)),
        new InstantCommand(() -> swerve.setXModeEnabled(true)));
  }
}