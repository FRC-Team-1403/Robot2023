package team1403.robot.chargedup;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.robot.chargedup.arm.ArmStateGroup;
import team1403.robot.chargedup.arm.ArmSubsystem;
import team1403.robot.chargedup.arm.RunIntake;
import team1403.robot.chargedup.arm.SequentialMoveArmCommand;
import team1403.robot.chargedup.arm.SetpointArmCommand;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

public class AutoManager {
  static private AutoManager m_instance;

  private ArrayList<PathPlannerTrajectory> onePiece = (ArrayList<PathPlannerTrajectory>) 
  PathPlanner.loadPathGroup("One Piece Auto", new PathConstraints(4, 3));;

  private ArrayList<PathPlannerTrajectory> twoPiece = (ArrayList<PathPlannerTrajectory>) 
  PathPlanner.loadPathGroup("Two Piece Auto", new PathConstraints(4, 3));

  private ArrayList<PathPlannerTrajectory> threePiece = (ArrayList<PathPlannerTrajectory>) 
  PathPlanner.loadPathGroup("Three Piece Auto", new PathConstraints(4, 3));;

  private HashMap<String, Command> eventMap = new HashMap<>();
  
  private final ProfiledPIDController thetaController = new ProfiledPIDController(
    RobotConfig.Swerve.kPAutoTurning,
    RobotConfig.Swerve.kIAutoTurning,
    RobotConfig.Swerve.kDAutoTurning,
    RobotConfig.Swerve.kThetaControllerConstraints);

  private CommandBase onePieceAuto;
  private CommandBase twoPieceAuto;
  private CommandBase threePieceAuto;

  private AutoManager() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public static AutoManager getInstance() {
    if (m_instance == null) {
      m_instance = new AutoManager();
    }
    return m_instance;
  }

  public void init(SwerveSubsystem swerve, ArmSubsystem arm) {
    //state.setIntakeSpeed(1.0);
    eventMap.put("lowCubeIntake", new SequentialMoveArmCommand(arm, () -> RobotConfig.ArmStates.cubeFloorIntake, false));
    //max speed is 1.0
    eventMap.put("tuck", new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false));
    final double armWait = 0.01;
    eventMap.put("highNodeCone", new SequentialMoveArmCommand(arm, () -> RobotConfig.ArmStates.coneHighNodeAuton, false)
                                                          .andThen(new WaitCommand(armWait)));
    eventMap.put("highNodeCube", new SequentialMoveArmCommand(arm, () -> RobotConfig.ArmStates.cubeHighNode, false)
                                                          .andThen(new WaitCommand(armWait)));
    eventMap.put("runOutake", new RunIntake(arm, 1.0, 1.0));
    eventMap.put("runIntake", new RunIntake(arm, -1.0, 2.0));
    eventMap.put("stop", new InstantCommand(() -> swerve.stop(), swerve));

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
    onePieceAuto = autoBuilder.fullAuto(onePiece).andThen(() -> swerve.stop(), swerve);
    twoPieceAuto = autoBuilder.fullAuto(twoPiece).andThen(() -> swerve.stop(), swerve);
    threePieceAuto = autoBuilder.fullAuto(threePiece).andThen(() -> swerve.stop(), swerve);
  }
  
  public CommandBase getThreePieceAuto(SwerveSubsystem swerve)
  {
    return new InstantCommand(() -> swerve.setSpeedLimiter(1.0), swerve).andThen(threePieceAuto);
  }

  public CommandBase getPathPlannerAuto(SwerveSubsystem swerve)
  {
    return new InstantCommand(() -> swerve.setSpeedLimiter(1.0), swerve).andThen(onePieceAuto);
  }

  public CommandBase getTwoPieceAuto(SwerveSubsystem swerve)
  {
    return new InstantCommand(() -> swerve.setSpeedLimiter(1.0), swerve).andThen(twoPieceAuto);
  }
}