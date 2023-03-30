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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  private final TrajectoryConfig m_trajectoryConfig1 = new TrajectoryConfig(4,
      2).setKinematics(RobotConfig.Swerve.kDriveKinematics);
    
  private final TrajectoryConfig m_trajectoryConfig2 = new TrajectoryConfig(8,
2).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_trajectoryConfig3 = new TrajectoryConfig(6,
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

  private SwerveControllerCommand sideGridTrajectory1;
  private SwerveControllerCommand sideGridTrajectory2;
  private SwerveControllerCommand sideGridTrajectory3;

  private SwerveControllerCommand sideGridStraightTrajectory1;
  private SwerveControllerCommand sideGridStraightTrajectory2;

  private SwerveControllerCommand sideGridTrajectory1Reverse;
  private SwerveControllerCommand sideGridTrajectory2Reverse;
  private SwerveControllerCommand sideGridTrajectory3Reverse;

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

    sideGridStraightTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, -0.3),
                new Translation2d(-2, -0.3)),
            new Pose2d(-5, -0.5, Rotation2d.fromDegrees(180)),
            m_trajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);
    
    sideGridStraightTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5, -0.5, Rotation2d.fromDegrees(180)),
            List.of(
                new Translation2d(-4, -0.5),
                new Translation2d(-3, 0)),
            new Pose2d(-0.2, -1.05, Rotation2d.fromDegrees(4)),
            m_trajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    sideGridTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, -0.3),
                new Translation2d(-2, -0.3)),
            new Pose2d(-3, 0, Rotation2d.fromDegrees(180)),
            m_trajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    sideGridTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(-3, 0, Rotation2d.fromDegrees(180)),
            List.of(
                new Translation2d(-4.7, 0.7),
                new Translation2d(-5.5, -0.4)),
            new Pose2d(-5.7, -0.7, Rotation2d.fromDegrees(270)),
            m_trajectoryConfig2),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    sideGridTrajectory3 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5.7, -0.7, Rotation2d.fromDegrees(270)),
            List.of(
                new Translation2d(-2.5, -0.5),
                new Translation2d(-0.5, -0.5)),
            new Pose2d(0.17, -1.05, Rotation2d.fromDegrees(4)),
            m_trajectoryConfig3),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    sideGridTrajectory1Reverse = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, 0.3),
                new Translation2d(-2, 0.3)),
            new Pose2d(-3, 0, Rotation2d.fromDegrees(180)),
            m_trajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    sideGridTrajectory2Reverse = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(-3, 0, Rotation2d.fromDegrees(180)),
            List.of(
                new Translation2d(-4.7, -0.7),
                new Translation2d(-5.05, 0.4)),
            new Pose2d(-5.1, 0.7, Rotation2d.fromDegrees(260)),
            m_trajectoryConfig2),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    sideGridTrajectory3Reverse = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5.05, 0.7, Rotation2d.fromDegrees(260)),
            List.of(
                new Translation2d(-2.5, 0.5),
                new Translation2d(-0.5, 0.5)),
            new Pose2d(0.14, 1.05, Rotation2d.fromDegrees(4)),
            m_trajectoryConfig3),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);
  }

  /**
   * Command for grids 3. Right grid is in relation to red alliance
   * 
   * @param swerve
   * @param arm
   * @return the command
   */
  public Command getRightGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    // swerve.setSpeedLimiter(1);
    // if (DriverStation.getAlliance() == Alliance.Red) {
    //     return new SequentialCommandGroup(
    //         new SequentialMoveArmCommand(arm,
    //         StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
    //         new RunIntake(arm, 1),
    //         new ParallelCommandGroup(
    //             new SequentialCommandGroup(
    //                 new WaitCommand(0.1),
    //                 new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
    //                 new WaitCommand(1.8),
    //                 new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
    //                 new SetpointArmCommand(arm, () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
    //                     true),
    //                 new RunIntake(arm, 3, 1.6),
    //                 new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
    //                 new WaitCommand(0.2),
    //                 new SetpointArmCommand(arm, () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false)
    //                 ),
    //             new SequentialCommandGroup(
    //                 sideGridTrajectory1,
    //                 sideGridTrajectory2,
    //                 sideGridTrajectory3)),
    //                 new RunIntake(arm, -1));

    //     return new SequentialCommandGroup(
    //         sideGridStraightTrajectory1,
    //         sideGridStraightTrajectory2);
    // }

    return new WaitCommand(0.2);

    //     return new SequentialCommandGroup(
    //         new ParallelCommandGroup(
    //             new SequentialCommandGroup(
    //                 new WaitCommand(0.6),
    //                 new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
    //                 new SetpointArmCommand(arm, () -> StateManager.getInstance.getCurrentArmGroup().getFloorIntakeState()),
    //                 new RunIntake(arm, 2, 1),
    //                 new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true)),
    //             new SequentialCommandGroup(
    //                 sideGridStraightTrajectory1,
    //                 sideGridStraightTrajectory2));  
    //         )      
    //     );
    // }
    // return new SequentialCommandGroup(
    //         new SequentialMoveArmCommand(arm,
    //         StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
    //         new RunIntake(arm, 1),
    //         new ParallelCommandGroup(
    //             new SequentialCommandGroup(
    //                 new WaitCommand(0.1),
    //                 new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
    //                 new WaitCommand(1.8),
    //                 new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
    //                 new SetpointArmCommand(arm, () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
    //                     true),
    //                 new RunIntake(arm, 3, 1.6),
    //                 new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
    //                 new WaitCommand(1.2),
    //                 new SetpointArmCommand(arm, () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false)
    //                 ),
    //             new SequentialCommandGroup(
    //                 sideGridTrajectory1Reverse,
    //                 sideGridTrajectory2Reverse,
    //                 sideGridTrajectory3Reverse)),
    //                 new RunIntake(arm, -1));
    }


  // In relation to red side
  public Command getLeftGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    if (DriverStation.getAlliance() == Alliance.Red) {
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
                    new RunIntake(arm, 3, 1.6),
                    new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                    new WaitCommand(1.2),
                    new SetpointArmCommand(arm, () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false)
                    ),
                new SequentialCommandGroup(
                    sideGridTrajectory1Reverse,
                    sideGridTrajectory2Reverse,
                    sideGridTrajectory3Reverse)),
                    new RunIntake(arm, -1));
    }
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
                new RunIntake(arm, 3, 1.6),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new WaitCommand(1.2),
                new SetpointArmCommand(arm, () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false)
                ),
            new SequentialCommandGroup(
                sideGridTrajectory1,
                sideGridTrajectory2,
                sideGridTrajectory3)),
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