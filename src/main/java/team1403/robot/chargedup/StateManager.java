package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class StateManager {
    public ArmPosition armPosition = ArmPosition.NA;
    public ScoringShelves scoringShelves = ScoringShelves.NA;
    public ScoringNodes scoringNodes = ScoringNodes.NA;
    public ScoringHybrid scoringHybrid = ScoringHybrid.NA;
    public Alliance alliance;

    public double armAngle;
    public double wristAngle;
    public double armExtension; 

    public GamePiece gamePiece = GamePiece.NA;

    public enum ArmPosition {
        LOW,
        MID,
        HIGH,
        SHELF,
        FLOOR,
        NA
    }
    
    public enum GamePiece {
        CUBE,
        CONESIDEWAYS,
        CONEAWAY,
        CONEUPRIGHT,
        CONETOWARDS,
        NA
    }

    public enum ScoringShelves {
        SHELF1,
        SHELF2,
        SHELF3,
        SHELF4,
        SHELF5,
        SHELF6,
        NA
    }

    public enum ScoringNodes {
        NODE1,
        NODE2,
        NODE3,
        NODE4,
        NODE5,
        NODE6,
        NODE7,
        NODE8,
        NA
    }

    public enum ScoringHybrid {
        HYBRID1,
        HYBRID2,
        HYBRID3,
        HYBRID4,
        HYBRID5,
        HYBRID6,
        HYBRID7,
        HYBRID8,
        HYBRID9,
        NA
    }

    private StateManager() {
        alliance = DriverStation.getAlliance();
    }

    public static StateManager getInstance() {
        if(inst == null) {
            inst = new StateManager();
        }
        return inst;
    }

    public void updateScoringShelf() {
        switch (scoringShelves) {
            case NA: 
                armAngle = 0;
                wristAngle = 0;
                armExtension = 0;
                break;
            case SHELF1:
                armAngle = 1;
                wristAngle = 1;
                armExtension = 1;
                break;
            case SHELF2:
                armAngle = 2;
                wristAngle = 2;
                armExtension = 2;
                break;
            case SHELF3:
                armAngle = 3;
                wristAngle = 3;
                armExtension = 3;
                break;
            case SHELF4:
                armAngle = 4;
                wristAngle = 4;
                armExtension = 4;
                break;
            case SHELF5:
                armAngle = 5;
                wristAngle = 5;
                armExtension = 5;
                break;
            case SHELF6:
                armAngle = 6;
                wristAngle = 6;
                armExtension = 6;
                break;           
            
        }   
    }

    private static StateManager inst = null;
}