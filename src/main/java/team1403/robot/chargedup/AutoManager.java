package team1403.robot.chargedup;

import java.util.ArrayList;
import java.util.HashMap;
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

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class AutoManager {
  static private AutoManager m_instance;

  private ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("TestingAuto", new PathConstraints(4, 3));

  private HashMap<String, Command> eventMap = new HashMap<>();

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
    4,
    RobotConfig.Swerve.kIAutoTurning,
    RobotConfig.Swerve.kDAutoTurning,
    RobotConfig.Swerve.kThetaControllerConstraints);

    private Command pathplannerAuto;

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
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        () -> swerve.getPose(), // Pose2d supplier
        pose -> swerve.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        RobotConfig.Swerve.kDriveKinematics, // SwerveDriveKinematics
        new PIDConstants( RobotConfig.Swerve.kPTranslation, RobotConfig.Swerve.kITranslation, RobotConfig.Swerve.kDTranslation ), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants( RobotConfig.Swerve.kPAutoTurning, RobotConfig.Swerve.kIAutoTurning, RobotConfig.Swerve.kDAutoTurning ), // PID constants to correct for rotation error (used to create the rotation controller)
        moduleStates -> swerve.setModuleStates(moduleStates), // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should th>e path be automatically mirrored depending on alliance color. Optional, defaults to true
        swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );
    pathplannerAuto = autoBuilder.fullAuto(pathGroup);
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

  public Command getOldRedRightGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
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

  public Command getOldBlueRightGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
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
  public Command getBlueRightGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
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

  public Command getStraightTrajectory(SwerveSubsystem swerve, ArmSubsystem arm) {
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