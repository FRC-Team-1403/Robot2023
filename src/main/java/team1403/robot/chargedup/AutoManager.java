package team1403.robot.chargedup;

import java.util.ArrayList;
import java.util.HashMap;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team1403.robot.chargedup.swerve.SwerveModule;
import team1403.robot.chargedup.swerve.SwerveSubsystem;
import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class AutoManager {
  static private AutoManager m_instance;

  private ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) 
  PathPlanner.loadPathGroup("New Path", new PathConstraints(4, 3));

  private HashMap<String, Command> eventMap = new HashMap<>();
  private SwerveSubsystem m_subsystem;

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
    RobotConfig.Swerve.kPAutoTurning,
    RobotConfig.Swerve.kIAutoTurning,
    RobotConfig.Swerve.kDAutoTurning,
    RobotConfig.Swerve.kThetaControllerConstraints);

    private CommandBase pathplannerAuto;

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
    m_subsystem = swerve;
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        () -> m_subsystem.getPose(), // Pose2d supplier
        pose -> m_subsystem.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        RobotConfig.Swerve.kDriveKinematics, // SwerveDriveKinematics
        new PIDConstants( RobotConfig.Swerve.kPTranslation, RobotConfig.Swerve.kITranslation, RobotConfig.Swerve.kDTranslation ), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants( RobotConfig.Swerve.kPAutoTurning, RobotConfig.Swerve.kIAutoTurning, RobotConfig.Swerve.kDAutoTurning ), // PID constants to correct for rotation error (used to create the rotation controller)
        moduleStates -> m_subsystem.setModuleStates(moduleStates), // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should th>e path be automatically mirrored depending on alliance color. Optional, defaults to true
        m_subsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );
    pathplannerAuto = autoBuilder.fullAuto(pathGroup).andThen(() -> m_subsystem.stop(), m_subsystem);
  } 


  public CommandBase getPathPlannerAuto(SwerveSubsystem swerve)
  {
    swerve.setSpeedLimiter(1.0);
    return pathplannerAuto;
  }
}