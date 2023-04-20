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
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
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
import team1403.robot.chargedup.swerve.SwerveAutoBalanceYaw;
import team1403.robot.chargedup.swerve.SwerveControllerCommand;
import team1403.robot.chargedup.swerve.SwerveSubsystem;
import team1403.robot.chargedup.swerve.TimedDrive;

public class AutoManager {
  static private AutoManager m_instance;

  private final TrajectoryConfig m_straightTrajectoryConfig = new TrajectoryConfig(3,
  3.0).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig redRight2PieceTrajectoryConfig1 = new TrajectoryConfig(14.5,
      3.25).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig blueRight2PieceTrajectoryConfig1 = new TrajectoryConfig(14.5,
      3).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig right2PieceTrajectoryConfig2 = new TrajectoryConfig(4,
      1).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig right3PieceTrajectoryConfig2 = new TrajectoryConfig(10,
      3).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_reverseTrajectoryConfig3 = new TrajectoryConfig(10,
      2).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig right3PieceTrajectoryConfig1 = new TrajectoryConfig(3,
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

  private SwerveControllerCommand redRight2PieceTrajectory1;
  private SwerveControllerCommand redRight2PieceTrajectory2;

  private SwerveControllerCommand blueRight2PieceTrajectory1;
  private SwerveControllerCommand blueRight2PieceTajectory2;

  private SwerveControllerCommand redRight3PieceTrajectory1;
  private SwerveControllerCommand redRight3PieceTrajectory2;
  private SwerveControllerCommand redRight3PieceTrajectory3;  
  private SwerveControllerCommand redRight3PieceTrajectory4;

  private SwerveControllerCommand redRight3PieceTrajectory2Copy;
  private SwerveControllerCommand blueSide3PieceTrajectory2Copy;

  private SwerveControllerCommand blueSide3PieceTrajectory1;
  private SwerveControllerCommand blueSide3PieceTrajectory2;
  private SwerveControllerCommand blueSide3PieceTrajectory3;

  private SwerveControllerCommand straightTrajectory1;

  private SwerveControllerCommand balanceTrajectory1;
  private SwerveControllerCommand balanceTrajectory2;

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
            m_straightTrajectoryConfig),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRight2PieceTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, -0.3),
                new Translation2d(-2, -0.3)),
            new Pose2d(-3, 0.1, Rotation2d.fromDegrees(-90)),
            redRight2PieceTrajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRight2PieceTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(-3, 0.1, Rotation2d.fromDegrees(-90)),
            List.of(
                new Translation2d(-5.0, 0.7),
                new Translation2d(-5.4, -0.3)),
            new Pose2d(-5.75, -0.7, Rotation2d.fromDegrees(-90)),
            right2PieceTrajectoryConfig2),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    blueRight2PieceTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, 0.3),
                new Translation2d(-2, 0.3)),
            new Pose2d(-3, 0, Rotation2d.fromDegrees(90)),
            blueRight2PieceTrajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    blueRight2PieceTajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory( 
            new Pose2d(-3, 0, Rotation2d.fromDegrees(90)),
            List.of(
                new Translation2d(-5, -0.7),
                new Translation2d(-5.5, 0.3)),
            new Pose2d(-5.5, 0.7, Rotation2d.fromDegrees(90)),
            right2PieceTrajectoryConfig2),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRight3PieceTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, -0.3),
                new Translation2d(-2, -0.4),
                new Translation2d(-3, 0.1),
                new Translation2d(-5.2, 0.7)),
            new Pose2d(-5.51, -0.7, Rotation2d.fromDegrees(-90)),
            right3PieceTrajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRight3PieceTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5.51, -0.7, Rotation2d.fromDegrees(-90)),
            List.of(
                new Translation2d(-5.55, -0.8),
                new Translation2d(-4.5, -0.8),
                new Translation2d(-2.5, -0.8),
                new Translation2d(-0.5, -0.9)),
            new Pose2d(0.05, -1.1, Rotation2d.fromDegrees(1)),
            right3PieceTrajectoryConfig2.setStartVelocity(5)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRight3PieceTrajectory3 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.05, -1.1, Rotation2d.fromDegrees(1)),
            List.of(
                new Translation2d(-0.5, -0.8),
                new Translation2d(-2.5, -0.9),
                new Translation2d(-4.5, -0.9),
                new Translation2d(-5.0, -1)),
            new Pose2d(-5.55, -2.22, Rotation2d.fromDegrees(-90)),
            right3PieceTrajectoryConfig2.setStartVelocity(5)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRight3PieceTrajectory4 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5.55, -2.22, Rotation2d.fromDegrees(-90)),
            List.of(
                new Translation2d(-4.5, -1.5),
                new Translation2d(-2.5, -1.5),
                new Translation2d(-0.5, -1.5)),
            new Pose2d(-0.05, -2.04, Rotation2d.fromDegrees(1)),
            right3PieceTrajectoryConfig2.setStartVelocity(5)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    blueSide3PieceTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, 0.3),
                new Translation2d(-2, 0.3),
                new Translation2d(-3, 0.1),
                new Translation2d(-5.2, -0.7)),
            new Pose2d(-5.65, 0.7, Rotation2d.fromDegrees(90)),
            right3PieceTrajectoryConfig1),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    blueSide3PieceTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5.65, 0.7, Rotation2d.fromDegrees(90)),
            List.of(
                new Translation2d(-5.7, 0.65),
                new Translation2d(-4.5, 0.65),
                new Translation2d(-2.5, 0.65),
                new Translation2d(-0.5, 0.8)),
            new Pose2d(0.04, 1.02, Rotation2d.fromDegrees(0)),
            right3PieceTrajectoryConfig2.setStartVelocity(5)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    blueSide3PieceTrajectory3 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.04, 1.02, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-0.5, 0.5),
                new Translation2d(-2.5, 0.55),
                new Translation2d(-4.5, 0.55),
                new Translation2d(-5.7, 0.55),
                new Translation2d(-5.96, 0.55)),
            new Pose2d(-5.96, 0.65, Rotation2d.fromDegrees(180)),
            right3PieceTrajectoryConfig2.setStartVelocity(5)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    balanceTrajectory1 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          List.of(
            new Translation2d(-3, 0)),
            new Pose2d(-5.1, 0, Rotation2d.fromDegrees(1)),
        new TrajectoryConfig(2, 3.25)
          .setKinematics(RobotConfig.Swerve.kDriveKinematics)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    balanceTrajectory2 = new SwerveControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-5.1, 0, Rotation2d.fromDegrees(1)),
            List.of(
            new Translation2d(-4.5, 0)),
            new Pose2d(-2.1, 0, Rotation2d.fromDegrees(2)),
        new TrajectoryConfig(2, 3.25)
            .setKinematics(RobotConfig.Swerve.kDriveKinematics)),
        swerve::getPose,
        xController,
        yController,
        thetaController,
        swerve);

    redRight3PieceTrajectory2Copy = redRight3PieceTrajectory2.copyOf();
    blueSide3PieceTrajectory2Copy = blueSide3PieceTrajectory2.copyOf();
  }
  /**
   * Red side right grid autonomous command. 
   * Score 2 pieces.
   */
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
                redRight2PieceTrajectory1,
                redRight2PieceTrajectory2,
                redRight3PieceTrajectory2Copy)));
  }

  /**
   * Blue side right grid autonomous command. 
   * Score 2 pieces.
   */
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
                blueRight2PieceTrajectory1,
                blueRight2PieceTajectory2,
                blueSide3PieceTrajectory2Copy)));
  }

  /**
   * Red side right grid autonomous command. 
   * Score 3 pieces.
   */
  public Command getRedRightGrid3PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new RunIntake(arm, -1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(2.37),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 0.9),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new WaitCommand(1.3),
                new RunIntake(arm, -1),
                new WaitCommand(1.67),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 1.2),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new WaitCommand(1.5),
                new RunIntake(arm, -1)),
            new SequentialCommandGroup(
                redRight3PieceTrajectory1,
                redRight3PieceTrajectory2,
                redRight3PieceTrajectory3,
                redRight3PieceTrajectory4)));
  }

  /**
    * Blue side right grid autonomous command. 
    * Score 3 pieces.
   */
  public Command getBlueRightGrid3PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new RunIntake(arm, -1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 1.2),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new WaitCommand(0.5),
                new RunIntake(arm, -1),
                new WaitCommand(0.5),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 1.2),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new WaitCommand(0.5),
                new RunIntake(arm, -1)),
            new SequentialCommandGroup(
                blueSide3PieceTrajectory1,
                blueSide3PieceTrajectory2,
                blueSide3PieceTrajectory3)));
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
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false)
            ),
            balanceTrajectory1
        ),
        new WaitCommand(0.1),
        balanceTrajectory2,
        new SwerveAutoBalanceYaw(swerve));
  }
}