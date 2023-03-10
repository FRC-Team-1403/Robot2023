package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.robot.chargedup.arm.ArmStateGroup;

public class StateManager {

  private Alliance alliance;

  private ArmStateGroup currentArmGroup;
  private ArmStateGroup cubeGroup;
  private ArmStateGroup coneUprightGroup;
  private ArmStateGroup coneTowardsGroup;
  private ArmStateGroup coneAwayGroup;

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

  private StateManager() {}

  public static StateManager getInstance() {
    if (instance == null) {
      instance = new StateManager();
    }
    return instance;
  }

  public void init() {
    alliance = DriverStation.getAlliance();
  }

  public void updateState(GamePiece newGamePiece) {
    this.gamePiece = newGamePiece;
    switch (newGamePiece) {
      case CUBE: {
        currentArmGroup = cubeGroup;
        break;
      }
       case CONE_UPRIGHT: {
        currentArmGroup = coneUprightGroup;
        break;
       }
       case CONE_AWAY: {
        currentArmGroup = coneAwayGroup;
        break;
       }
       case CONE_TOWARDS: {
        currentArmGroup = coneTowardsGroup;
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

  public Alliance getAlliance() {
    return alliance;
  }

  public ArmStateGroup getCurrentArmGroup() {
    return currentArmGroup;
  }

  public GamePiece getGamePiece() {
    return gamePiece;
  }

}