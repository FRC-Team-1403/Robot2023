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
import team1403.robot.chargedup.StateManager.GamePiece;
import team1403.robot.chargedup.arm.ArmState;
import team1403.robot.chargedup.arm.ArmStateGroup;
import team1403.robot.chargedup.arm.ArmSubsystem;
import team1403.robot.chargedup.arm.RunIntake;
import team1403.robot.chargedup.arm.SetpointArmCommand;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

public class AutoManager {
  static private AutoManager m_instance;

  private ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) 
  PathPlanner.loadPathGroup("One Piece Auto", new PathConstraints(4, 3));;

  private ArrayList<PathPlannerTrajectory> twoPiece = (ArrayList<PathPlannerTrajectory>) 
  PathPlanner.loadPathGroup("two piece auto 2.0", new PathConstraints(4, 3));

  private ArrayList<PathPlannerTrajectory> threePiece = (ArrayList<PathPlannerTrajectory>) 
  PathPlanner.loadPathGroup("Three Piece Auto", new PathConstraints(4, 3));;

  private HashMap<String, Command> eventMap = new HashMap<>();
  
  private final ProfiledPIDController thetaController = new ProfiledPIDController(
    RobotConfig.Swerve.kPAutoTurning,
    RobotConfig.Swerve.kIAutoTurning,
    RobotConfig.Swerve.kDAutoTurning,
    RobotConfig.Swerve.kThetaControllerConstraints);

  private CommandBase pathplannerAuto;
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
    StateManager.getInstance().updateArmState(GamePiece.CUBE);
    ArmState state = StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState();
    //state.setIntakeSpeed(1.0);
    eventMap.put("lowCubeIntake", new SetpointArmCommand(arm, () -> state, false));
    //max speed is 1.0
    eventMap.put("tuck", new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false));

    StateManager.getInstance().updateArmState(GamePiece.CONE_UPRIGHT);
    ArmState state2 = StateManager.getInstance().getCurrentArmGroup().getHighNodeState();

    eventMap.put("highNodeCone", new SetpointArmCommand(arm, () -> state2, false).andThen(new WaitCommand(0.5)));
    StateManager.getInstance().updateArmState(GamePiece.CUBE);
    eventMap.put("highNodeCube", new SetpointArmCommand(arm, () -> StateManager.getInstance().getCurrentArmGroup().
                                                          getHighNodeState(), false).andThen(new WaitCommand(0.5)));
    eventMap.put("runOutake", new RunIntake(arm, 1.0, 1.0));
    eventMap.put("runIntake", new RunIntake(arm, -1.0, 2.0));

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
    pathplannerAuto = autoBuilder.fullAuto(pathGroup).andThen(() -> swerve.stop(), swerve);
    twoPieceAuto = autoBuilder.fullAuto(twoPiece).andThen(() -> swerve.stop(), swerve);
    threePieceAuto = autoBuilder.fullAuto(threePiece).andThen(() -> swerve.stop(), swerve);
  }
  
  public CommandBase getThreePieceAuto(SwerveSubsystem swerve)
  {
    return new InstantCommand(() -> swerve.setSpeedLimiter(1.0), swerve).andThen(threePieceAuto);
  }

  public CommandBase getPathPlannerAuto(SwerveSubsystem swerve)
  {
    return new InstantCommand(() -> swerve.setSpeedLimiter(1.0), swerve).andThen(pathplannerAuto);
  }

  public CommandBase getTwoPieceAuto(SwerveSubsystem swerve)
  {
    return new InstantCommand(() -> swerve.setSpeedLimiter(1.0), swerve).andThen(twoPieceAuto);
  }
}