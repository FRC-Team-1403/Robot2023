package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.robot.chargedup.arm.ArmState;
import team1403.robot.chargedup.arm.ArmStateGroup;

public class StateManager {

  private Alliance alliance;

  private ArmStateGroup currentArmGroup;
  private ArmStateGroup cubeGroup;
  private ArmStateGroup coneUprightGroup;
  private ArmStateGroup coneTowardsGroup;
  private ArmStateGroup coneAwayGroup;

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
//Pivot 150.28003026
//Wrist 246.78781366
//Length 22.987735748
  private StateManager() {
    ArmState floorIntake = new ArmState(0.047618567943573, 133.59195783979897, 231.71438088502362, 1);
    ArmState highConeNode = new ArmState(22.987735748, 246.78781366, 150.28003026, 0);
    currentArmGroup = new ArmStateGroup(floorIntake, null, null, highConeNode, null, null);
  }

  public static StateManager getInstance() {
    if (instance == null) {
      instance = new StateManager();
    }
    return instance;
  }

  public void init() {
    alliance = DriverStation.getAlliance();
  }

  public void updateState(GamePiece newGamePiece, Runnable switchPipeline) {
    switchPipeline.run();
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
       case NA: {
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