package team1403.lib.util;

public class ArmGroup {
    private ArmState floorIntakeState;
    private ArmState doubleShelfIntakeState;
    private ArmState singleShelfIntakeState;
    private ArmState highNodeState;
    private ArmState middleNodeState;
    private ArmState lowNodeState;
    public static ArmState tuck = new ArmState(0, 0, 0, 0, null); //random values for now

    public ArmGroup(ArmState floor, ArmState doubleShelf, ArmState singleShelf, ArmState highNode, ArmState middleNode, ArmState lowNode) {
        this.floorIntakeState = floor;
        this.doubleShelfIntakeState = doubleShelf;
        this.singleShelfIntakeState = singleShelf;
        this.highNodeState = highNode;
        this.middleNodeState = middleNode;
        this.lowNodeState = lowNode;
    }

    public ArmState getFloorIntakeState() {
        return floorIntakeState;
    }

    public ArmState getDoubleShelfIntakeState() {
        return doubleShelfIntakeState;
    }

    public ArmState getSingleShelfIntakeState() {
        return singleShelfIntakeState;
    }

    public ArmState getHighNodeState() {
        return highNodeState;
    }

    public ArmState getMiddleNodeState() {
        return middleNodeState;
    }

    public ArmState getLowNodeState() {
        return lowNodeState;
    }

    public static ArmState getTuck() {
        return tuck;
    }


}




//Arm Group: 7 arm states for each kind of arm group
//Types of Arm groups: Cube, Cone Upright, Cone Toward, Cone away
//States: Tuck, low, medium, high, shelf, floor
//set method that changes the enum of what game piece is deteched and the orientation (for cone)
// getArmGroup: what arm group it is for whatever the enum detected