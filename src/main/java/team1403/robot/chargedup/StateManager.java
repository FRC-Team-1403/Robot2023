package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.robot.chargedup.RobotConfig.ArmStateConfig;
import team1403.robot.chargedup.arm.ArmStateGroup;

public class StateManager {

  private Alliance m_alliance;

  private ArmStateGroup m_currentArmGroup;
  private ArmStateGroup m_cubeGroup;
  private ArmStateGroup m_coneUprightGroup;
  private ArmStateGroup m_coneTowardsGroup;
  private ArmStateGroup m_coneAwayGroup;

  private GamePiece gamePiece = GamePiece.NA;

  public enum GamePiece {
    CUBE,
    CONE_SIDEWAYS,
    CONE_AWAY,
    CONE_UPRIGHT,
    CONE_TOWARDS,
    NA
  }

  private static StateManager instance = null;

  private StateManager() {
    m_coneTowardsGroup = new ArmStateGroup(ArmStateConfig.coneTowardsFloorIntake, null, 
        ArmStateConfig.singleSubstationIntake, ArmStateConfig.coneTowardsHighConeNode, 
        ArmStateConfig.coneTowardsMiddleNode, ArmStateConfig.coneTowardsLowNode);

    m_cubeGroup = new ArmStateGroup(ArmStateConfig.cubeFloorIntake, null, 
        ArmStateConfig.singleSubstationIntake, ArmStateConfig.cubeHighNode, 
        ArmStateConfig.cubeMiddleNode, ArmStateGroup.tuck);

    m_coneUprightGroup = new ArmStateGroup(null, null, 
        ArmStateConfig.singleSubstationIntake, ArmStateConfig.coneTowardsHighConeNode, 
        ArmStateConfig.coneTowardsMiddleNode, ArmStateConfig.coneTowardsLowNode);

    m_currentArmGroup = m_cubeGroup;
  }
  public static StateManager getInstance() {
    if (instance == null) {
      instance = new StateManager();
    }
    return instance;
  }

  public void init() {
    m_alliance = DriverStation.getAlliance();
  }

  public void updateState(GamePiece newGamePiece, Runnable switchPipeline) {
    switchPipeline.run();
    this.gamePiece = newGamePiece;
    switch (newGamePiece) {
      case CUBE: {
        m_currentArmGroup = m_cubeGroup;
        break;
      }
       case CONE_UPRIGHT: {
        m_currentArmGroup = m_coneUprightGroup;
        break;
       }
       case CONE_AWAY: {
        m_currentArmGroup = m_coneAwayGroup;
        break;
       }
       case CONE_TOWARDS: {
        m_currentArmGroup = m_coneTowardsGroup;
        break;
       }
       case CONE_SIDEWAYS: {
        SmartDashboard.putString("Operator Message", "Sideways cone found. Cannot intake.");
        break;
       }
       case NA: {
        SmartDashboard.putString("Operator Message", "No game piece found.");
        break;
       }
    }
  }

  public Alliance getM_alliance() {
    return m_alliance;
  }

  public ArmStateGroup getM_currentArmGroup() {
    return m_currentArmGroup;
  }

  public GamePiece getGamePiece() {
    return gamePiece;
  }

}