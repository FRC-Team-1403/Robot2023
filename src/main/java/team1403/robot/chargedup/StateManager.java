package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.robot.chargedup.RobotConfig.ArmStates;
import team1403.robot.chargedup.arm.ArmStateGroup;

public class StateManager {

  private Alliance m_alliance;

  private ArmStateGroup m_currentArmGroup;
  private ArmStateGroup m_cubeGroup;
  private ArmStateGroup m_coneUprightGroup;
  private ArmStateGroup m_coneTowardsGroup;
  private ArmStateGroup m_coneAwayGroup;

  private GamePiece gamePiece = GamePiece.NONE;

  public enum GamePiece {
    CUBE,
    CONE_UPRIGHT,
    CONE_AWAY, 
    CONE_SIDEWAYS, 
    CONE_TOWARDS,
    NONE;

    static final GamePiece[] values = GamePiece.values();

    public static GamePiece fromInt(int value) {
      switch(value) {
        case 4: // a cube has 4 sides :p
          return CUBE;
        case 117: // 'u'
          return CONE_UPRIGHT;
        case 115: // 's'
          return CONE_SIDEWAYS;
        case 104: // 'h'
          return CONE_TOWARDS;
        case 97:  // 'a'
          return CONE_AWAY;
        default: // nothing's there :O
          return NONE;
      }
    }
  }

  private static StateManager instance = null;

  private StateManager() {
    m_coneTowardsGroup = new ArmStateGroup(ArmStates.coneTowardsFloorIntake, null, 
        ArmStates.singleSubstationIntake, ArmStates.coneTowardsHighConeNode, 
        ArmStates.coneTowardsMiddleNode, ArmStates.coneTowardsLowNode);

    m_cubeGroup = new ArmStateGroup(ArmStates.cubeFloorIntake, null, 
        ArmStates.singleSubstationIntake, ArmStates.cubeHighNode, 
        ArmStates.cubeMiddleNode, ArmStateGroup.tuck);

    m_coneUprightGroup = new ArmStateGroup(ArmStates.coneUprightIntake, null, 
        ArmStates.singleSubstationIntake, ArmStates.coneTowardsHighConeNode, 
        ArmStates.coneTowardsMiddleNode, ArmStates.coneTowardsLowNode);

    m_currentArmGroup = m_coneTowardsGroup;
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

  public void updateState(GamePiece newGamePiece) {
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
       case NONE: {
        SmartDashboard.putString("Operator Message", "No game piece found.");
        break;
       }
    }
  }

  public Alliance getM_alliance() {
    return m_alliance;
  }

  public ArmStateGroup getCurrentArmGroup() {
    return m_currentArmGroup;
  }

  public GamePiece getGamePiece() {
    return gamePiece;
  }

}