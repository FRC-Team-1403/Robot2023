package team1403.robot.chargedup;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.robot.chargedup.RobotConfig.Arm;
import team1403.robot.chargedup.StateManager.GamePiece;
import team1403.robot.chargedup.arm.ArmStateGroup;
import team1403.robot.chargedup.arm.ArmSubsystem;
import team1403.robot.chargedup.arm.RunIntake;
import team1403.robot.chargedup.arm.SequentialMoveArmCommand;
import team1403.robot.chargedup.arm.SetpointArmCommand;
import team1403.robot.chargedup.swerve.SwerveAutoBalanceYaw;
import team1403.robot.chargedup.swerve.SwerveControllerCommand;
import team1403.robot.chargedup.swerve.SwerveSubsystem;
import team1403.robot.chargedup.swerve.TimedDrive;

public class AutoManager {
  static private AutoManager m_instance;

  private final TrajectoryConfig m_straightTrajectory = new TrajectoryConfig(3,
  3.0).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_trajectoryConfig1 = new TrajectoryConfig(14.5,
      3.25).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_reverseTrajectoryConfig1 = new TrajectoryConfig(14.5,
      3).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_trajectoryConfig2 = new TrajectoryConfig(4,
      1).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_trajectoryConfig3 = new TrajectoryConfig(10,
      3).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_reverseTrajectoryConfig3 = new TrajectoryConfig(10,
      2).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_combinedTrajectoryConfig = new TrajectoryConfig(3,
       3.25).setKinematics(RobotConfig.Swerve.kDriveKinematics).addConstraint(
        new RectangularRegionConstraint(new Translation2d(1, 1),
            new Translation2d(-1, -5),
            new SwerveDriveKinematicsConstraint(RobotConfig.Swerve.kDriveKinematics, 4)))
    .addConstraint(
        new RectangularRegionConstraint(new Translation2d(-4.5, 1),
        new Translation2d(-6, -5),
        new SwerveDriveKinematicsConstraint(RobotConfig.Swerve.kDriveKinematics, 4)));

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

  private SwerveControllerCommand oldRedRightGridTrajectory1;
  private SwerveControllerCommand oldRedRightGridTrajectory2;

  private SwerveControllerCommand oldBlueRightGridTrajectory1;
  private SwerveControllerCommand oldBlueRightGridTrajectory2;

  private SwerveControllerCommand redRightGridTrajectory1;
  private SwerveControllerCommand redRightGridTrajectory2;
  private SwerveControllerCommand redRightGridTrajectory3;  

  private SwerveControllerCommand redRightGridTrajectory2Copy;
  private SwerveControllerCommand blueSideGridTrajectory2Copy;

  private SwerveControllerCommand blueSideGridTrajectory1;
  private SwerveControllerCommand blueSideGridTrajectory2;
  private SwerveControllerCommand blueSideGridTrajectory3;

  private SwerveControllerCommand straightTrajectory1;

  private SwerveControllerCommand balanceTrajectory;

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

    straightTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1.5, 0)),
            new Pose2d(-4.3, 0, Rotation2d.fromDegrees(1)),
            m_straightTrajectory),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    oldRedRightGridTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, -0.3),
                new Translation2d(-2, -0.3)),
            new Pose2d(-3, 0.1, Rotation2d.fromDegrees(-90)),
            m_trajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    oldRedRightGridTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(-3, 0.1, Rotation2d.fromDegrees(-90)),
            List.of(
                new Translation2d(-5.0, 0.7),
                new Translation2d(-5.4, -0.3)),
            new Pose2d(-5.75, -0.7, Rotation2d.fromDegrees(-90)),
            m_trajectoryConfig2),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    oldBlueRightGridTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, 0.3),
                new Translation2d(-2, 0.3)),
            new Pose2d(-3, 0, Rotation2d.fromDegrees(90)),
            m_reverseTrajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    oldBlueRightGridTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(-3, 0, Rotation2d.fromDegrees(90)),
            List.of(
                new Translation2d(-5, -0.7),
                new Translation2d(-5.5, 0.3)),
            new Pose2d(-5.5, 0.7, Rotation2d.fromDegrees(90)),
            m_trajectoryConfig2),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRightGridTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, -0.3),
                new Translation2d(-2, -0.3),
                new Translation2d(-3, 0.1),
                new Translation2d(-5.2, 0.7)),
            new Pose2d(-5.50, -0.7, Rotation2d.fromDegrees(-90)),
            m_combinedTrajectoryConfig),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRightGridTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5.65, -0.7, Rotation2d.fromDegrees(-90)),
            List.of(
                new Translation2d(-5.7, -0.55),
                new Translation2d(-4.5, -0.55),
                new Translation2d(-2.5, -0.55),
                new Translation2d(-0.5, -0.7)),
            new Pose2d(0.05, -1.02, Rotation2d.fromDegrees(1)),
            m_trajectoryConfig3.setStartVelocity(5)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRightGridTrajectory3 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.05, -1.02, Rotation2d.fromDegrees(1)),
            List.of(
                new Translation2d(-0.5, -0.55),
                new Translation2d(-2.5, -0.55),
                new Translation2d(-4.5, -0.55),
                new Translation2d(-5.8, -0.55)),
            new Pose2d(-5.8, -0.65, Rotation2d.fromDegrees(-180)),
            m_trajectoryConfig3.setStartVelocity(5)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    blueSideGridTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, 0.3),
                new Translation2d(-2, 0.3),
                new Translation2d(-3, 0.1),
                new Translation2d(-5.2, -0.7)),
            new Pose2d(-5.65, 0.7, Rotation2d.fromDegrees(90)),
            m_combinedTrajectoryConfig),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    blueSideGridTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5.65, 0.7, Rotation2d.fromDegrees(90)),
            List.of(
                new Translation2d(-5.7, 0.65),
                new Translation2d(-4.5, 0.65),
                new Translation2d(-2.5, 0.65),
                new Translation2d(-0.5, 0.8)),
            new Pose2d(0.04, 1.02, Rotation2d.fromDegrees(0)),
            m_trajectoryConfig3.setStartVelocity(5)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    blueSideGridTrajectory3 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.04, 1.02, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-0.5, 0.5),
                new Translation2d(-2.5, 0.55),
                new Translation2d(-4.5, 0.55),
                new Translation2d(-5.7, 0.55),
                new Translation2d(-5.96, 0.55)),
            new Pose2d(-5.96, 0.65, Rotation2d.fromDegrees(180)),
            m_trajectoryConfig3.setStartVelocity(5)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    balanceTrajectory = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
          List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(-2.5, 0, Rotation2d.fromDegrees(1))),
        new TrajectoryConfig(6,
        3.25)
          .setKinematics(RobotConfig.Swerve.kDriveKinematics)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRightGridTrajectory2Copy = redRightGridTrajectory2.copyOf();
    blueSideGridTrajectory2Copy = blueSideGridTrajectory2.copyOf();
  }

  // Red alliance path involving a swing
  public Command getRedRightGrid2PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.65),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 1.2),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(),
                    false),
                new RunIntake(arm, -1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.05),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CONE_TOWARDS))),
            new SequentialCommandGroup(
                redRightGridTrajectory1,
                redRightGridTrajectory2,
                new WaitCommand(0.5),
                redRightGridTrajectory3)));
  }

  public Command getRedRightGrid1_5PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.45),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 3.85),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new SetpointArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(),
                    false)),
            new SequentialCommandGroup(
                oldRedRightGridTrajectory1,
                oldRedRightGridTrajectory2,
                redRightGridTrajectory2Copy)));
  }

  public Command getBlueRightGrid1_5PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.45),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 3.85),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new WaitCommand(0.01),
                new SetpointArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(),
                    false)),
            new SequentialCommandGroup(
                oldBlueRightGridTrajectory1,
                oldBlueRightGridTrajectory2,
                blueSideGridTrajectory2Copy)));
  }


  // Blue alliance path involving a swing
  public Command getBlueRightGrid2PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.65),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 1.3),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(),
                    false),
                new RunIntake(arm, -1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.04),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CONE_TOWARDS))),
            new SequentialCommandGroup(
                blueSideGridTrajectory1,
                blueSideGridTrajectory2,
                new WaitCommand(0.5),
                blueSideGridTrajectory3)));
  }

  public Command getTimedSideGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new TimedDrive(swerve, 0.8, new ChassisSpeeds(-3, 0, 0)),
        new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
        new TimedDrive(swerve, 6.5, new ChassisSpeeds(-4, 0, 0)));
  }

  public Command get1PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
        straightTrajectory1);
  }

  public Command getMiddleGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false)
            ),
            balanceTrajectory
        ),
        new SwerveAutoBalanceYaw(swerve));
  }
}