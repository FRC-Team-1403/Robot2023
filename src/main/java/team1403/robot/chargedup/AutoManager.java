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

  private final TrajectoryConfig m_trajectoryConfig1 = new TrajectoryConfig(14.5,
      3.25).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_reverseTrajectoryConfig1 = new TrajectoryConfig(14.5,
      3).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_trajectoryConfig2 = new TrajectoryConfig(4,
      1).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_trajectoryConfig3 = new TrajectoryConfig(9,
      2.75).setKinematics(RobotConfig.Swerve.kDriveKinematics);

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

  private SwerveControllerCommand redRightGridTrajectory1;
  private SwerveControllerCommand redRightGridTrajectory2;

  private SwerveControllerCommand redRightGridCombinedTrajectory;

  private SwerveControllerCommand redRightGridTrajectory3;
  private SwerveControllerCommand redRightGridTrajectory4;  

  private SwerveControllerCommand blueSideGridTrajectory1;
  private SwerveControllerCommand blueSideGridTrajectory2;
  private SwerveControllerCommand blueSideGridTrajectory3;

  private SwerveControllerCommand redRightGridTrajectory;

  private SwerveControllerCommand straightTrajectory;

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
    redRightGridCombinedTrajectory = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, -0.3),
                new Translation2d(-2, -0.3),
                new Translation2d(-3, 0.1),
                new Translation2d(-4.92, 0.7)),
            new Pose2d(-5.5, -0.7, Rotation2d.fromDegrees(-90)),
            m_combinedTrajectoryConfig),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRightGridTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(-3, 0.1, Rotation2d.fromDegrees(-90)),
            List.of(
                new Translation2d(-5.0, 0.7),
                new Translation2d(-5.4, -0.3)),
            new Pose2d(-5.5, -0.7, Rotation2d.fromDegrees(-90)),
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
                new Translation2d(-2, -0.3)),
            new Pose2d(-3, 0.1, Rotation2d.fromDegrees(-90)),
            m_trajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRightGridTrajectory3 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5.5, -0.7, Rotation2d.fromDegrees(-90)),
            List.of(
                new Translation2d(-5.7, -0.35),
                new Translation2d(-4.5, -0.35),
                new Translation2d(-2.5, -0.35),
                new Translation2d(-0.5, -0.5)),
            new Pose2d(0.35, -1.02, Rotation2d.fromDegrees(6)),
            m_trajectoryConfig3.setStartVelocity(5)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRightGridTrajectory4 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.35, -1.02, Rotation2d.fromDegrees(6)),
            List.of(
                new Translation2d(-0.5, -0.5),
                new Translation2d(-2.5, -0.35),
                new Translation2d(-4.5, -0.35),
                new Translation2d(-5.7, -0.35)),
            new Pose2d(-5.45, -0.7, Rotation2d.fromDegrees(179)),
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
                new Translation2d(-2, 0.3)),
            new Pose2d(-3, 0, Rotation2d.fromDegrees(90)),
            m_reverseTrajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    blueSideGridTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(-3, 0, Rotation2d.fromDegrees(90)),
            List.of(
                new Translation2d(-5.0, -0.7),
                new Translation2d(-5.5, 0.3)),
            new Pose2d(-5.5, 0.7, Rotation2d.fromDegrees(90)),
            m_trajectoryConfig2),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    blueSideGridTrajectory3 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5.5, 0.7, Rotation2d.fromDegrees(90)),
            List.of(
                new Translation2d(-5.7, 0.45),
                new Translation2d(-4.5, 0.45),
                new Translation2d(-2.5, 0.45),
                new Translation2d(-0.5, 0.55)),
            new Pose2d(0.10, 0.98, Rotation2d.fromDegrees(0)),
            m_reverseTrajectoryConfig3),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRightGridTrajectory = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(-1, -0.3, Rotation2d.fromDegrees(-1)),
                new Pose2d(-2, -0.3, Rotation2d.fromDegrees(-1)),
                new Pose2d(-3, 0.1, Rotation2d.fromDegrees(-1)),
                new Pose2d(-5.0, 0.7, Rotation2d.fromDegrees(-1)),
                new Pose2d(-5.5, -0.3, Rotation2d.fromDegrees(-1)),
                new Pose2d(-5.5, -0.7, Rotation2d.fromDegrees(-1)),
                new Pose2d(-5.7, -0.35, Rotation2d.fromDegrees(-1)),
                new Pose2d(-4.5, -0.35, Rotation2d.fromDegrees(-1)),
                new Pose2d(-2.5, -0.35, Rotation2d.fromDegrees(-1)),
                new Pose2d(-0.5, -0.5, Rotation2d.fromDegrees(0)),
                new Pose2d(0.15, -0.98, Rotation2d.fromDegrees(0))),
            new TrajectoryConfig(14.5,
                3.25)
                .setKinematics(RobotConfig.Swerve.kDriveKinematics)
                .addConstraint(
                    new RectangularRegionConstraint(new Translation2d(1, 1),
                        new Translation2d(-1, -5),
                        new SwerveDriveKinematicsConstraint(RobotConfig.Swerve.kDriveKinematics, 4)))
                .addConstraint(
                    new RectangularRegionConstraint(new Translation2d(-4.5, 1),
                    new Translation2d(-6, -5),
                    new SwerveDriveKinematicsConstraint(RobotConfig.Swerve.kDriveKinematics, 4)))),
            swerve::getPose,
            xController,
            yController,
            thetaController,
            swerve);

    straightTrajectory = new SwerveControllerCommand(
      TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(-0.01, 0, Rotation2d.fromDegrees(0)),
          new Pose2d(-5.5, 0, Rotation2d.fromDegrees(90)),
          new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
      new TrajectoryConfig(1,
      2)
        .setKinematics(RobotConfig.Swerve.kDriveKinematics)
        .addConstraint(
          new RectangularRegionConstraint(new Translation2d(1, 1),
            new Translation2d(-1, -5),
            new SwerveDriveKinematicsConstraint(RobotConfig.Swerve.kDriveKinematics, 4)))
        .addConstraint(
          new RectangularRegionConstraint(new Translation2d(-4.5, 1),
          new Translation2d(-6, -5),
          new SwerveDriveKinematicsConstraint(RobotConfig.Swerve.kDriveKinematics, 4)))),
      swerve::getPose,
      xController,
      yController,
      thetaController,
      swerve);

    balanceTrajectory = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
          List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(-3.5, 0, Rotation2d.fromDegrees(1))),
        new TrajectoryConfig(6,
        3.25)
          .setKinematics(RobotConfig.Swerve.kDriveKinematics)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);
  }

  // Red alliance path involving a swing
  public Command getRedRightGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
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
                new RunIntake(arm, 3, 1.2),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new WaitCommand(0.1),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(),
                    false),
                    new RunIntake(arm, -1),
                    new WaitCommand(0.1),
                    new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false)),
            new SequentialCommandGroup(
                redRightGridCombinedTrajectory,
                redRightGridTrajectory3,
                redRightGridTrajectory4)));
  }

  // Blue alliance path involving a swing
  public Command getBlueRightGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new WaitCommand(0.45),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 3, 3.85),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new WaitCommand(0.05),
                new SetpointArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(),
                    false)),
            new SequentialCommandGroup(
                blueSideGridTrajectory1,
                blueSideGridTrajectory2,
                blueSideGridTrajectory3))
    // new RunIntake(arm, -1)
    );
  }

  public Command getTimedSideGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
        new RunIntake(arm, 1),
        new TimedDrive(swerve, 1, new ChassisSpeeds(-2, 0, 0)),
        new ParallelCommandGroup(
            new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
            new TimedDrive(swerve, 6.5, new ChassisSpeeds(-2, 0, 0))));
  }

  public Command getImprovedRedSideGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return redRightGridTrajectory;
  }

  public Command getImprovedStraightCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return straightTrajectory;
  }
}