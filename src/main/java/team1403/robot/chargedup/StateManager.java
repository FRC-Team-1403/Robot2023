package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.robot.chargedup.RobotConfig.Arm;
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
//Wrist 140.0363630009091
    //Arm Pivot 240.55448872511047
    //Arm length 0.039682067930698
  private StateManager() {
    ArmState coneTowardsFloorIntake = new ArmState(0.039682067930698, 140.0363630009091, 240.55448872511047, 0);
    ArmState coneTowardsHighConeNode = new ArmState(22.987735748, 246.78781366, 150.28003026, 0);
    ArmState coneTowardsMiddleNode = new ArmState(8.345230102539062, 264, 149.45086251051157, 0);
    ArmState singleSubstationIntake = new ArmState(0, 51.3175107829, 241.777313195, 0);
    ArmState coneTowardsLowNode = new ArmState(0, 80.18787350469682, 245.42271036546947, 0);

    coneTowardsGroup = new ArmStateGroup(coneTowardsFloorIntake, null, singleSubstationIntake, coneTowardsHighConeNode, coneTowardsMiddleNode, coneTowardsLowNode);

    ArmState cubeFloorIntake = new ArmState(4.055530548095703, 124.90037862250946, 230.28624941341906, 0);
    ArmState cubeHighNode = new ArmState(16.6710987091, 177.965261949, 169.356773014, 0);
    ArmState cubeMiddleNode = new ArmState(0.05158682167, 177.61028394, 181.482400676, 0);
    
    cubeGroup = new ArmStateGroup(cubeFloorIntake, null, singleSubstationIntake, cubeHighNode, cubeMiddleNode, ArmStateGroup.tuck);
    currentArmGroup = cubeGroup;
  }0
1
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