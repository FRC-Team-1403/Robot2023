package team1403.robot.chargedup.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.chargedup.StateManager;
import team1403.robot.chargedup.StateManager.GamePiece;

public class UpdateArmState extends CommandBase{

    public UpdateArmState(GamePiece gamePiece) {
        StateManager.getInstance().updateArmState(gamePiece);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
