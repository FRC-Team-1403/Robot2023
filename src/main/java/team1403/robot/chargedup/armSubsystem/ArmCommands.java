 
package team1403.robot.chargedup.armSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmCommands extends CommandBase{

    /* TODO
     * Joystick for telescopic arm to extend, ask person
     * Joystick for telescopic arm to angle, ask person
     * 4 LIMIT SWITCHES??!??!?!?!?!?!!?!?!?!?
     */

    private double maxThreshold = 270; //TODO set to the actual max threshold
    private double minThreshold = 45; //TODO set to the actual min threshold

    private final XboxController xBoxControllerInput;
    private final Arm m_arm;

    public ArmCommands(Arm spinny, XboxController controller) {
        this.m_arm = spinny;
        this.xBoxControllerInput = controller;
        addRequirements(spinny); //"Locks the subsystem to the command"
    }

    @Override
    public void execute() {

        double resultForAngle = this.m_arm.getArmAngle();
        double resultForAmps = this.m_arm.getCurrentAmps();

        //First Checkpoint
        if(m_arm.isFrontSwitch()) {
            m_arm.setAngledMotorSpeed(0);
        } else if(m_arm.isBackSwitch()) {
            m_arm.setAngledMotorSpeed(0);
        }
 
        //Second Checkpoint
        //TODO CHECK IF DELAY IS NEGLIGIBLE OR NOT
        if (resultForAngle >= maxThreshold) {
            m_arm.setAngledMotorSpeed(0);
        } else if (resultForAngle <= minThreshold) {
            m_arm.setAngledMotorSpeed(0);
        }

        //Third Checkpoint
        if(resultForAmps >= 15) {
            m_arm.setAngledMotorSpeed(0);
        }

        //Joystick Input
        xBoxControllerInput.getLeftY(); //y is anglation
        //System.out.println(xBoxControllerInput.getLeftY());
        xBoxControllerInput.getRightX(); //x is extension
    }
}
