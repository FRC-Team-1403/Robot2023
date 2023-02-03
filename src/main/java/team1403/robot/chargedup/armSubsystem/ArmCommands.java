 
package team1403.robot.chargedup.armSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * class ArmCommands is the where the commands for Arm.java is located
 */
public class ArmCommands extends CommandBase {

  /* TODO
     * Joystick for telescopic arm to extend, ask person
     * Joystick for telescopic arm to angle, ask person
   */

  private double maxThreshold = 270; //TODO set to the actual max threshold
  private double minThreshold = 45; //TODO set to the actual min threshold

  private final XboxController xBoxControllerInput;
  private final Arm m_arm;

  /**
   * defines spinny, as m_arm.
   * sets xBoxControllerInput to controller
   */
  public ArmCommands(Arm spinny, XboxController controller) {
    this.m_arm = spinny;
    this.xBoxControllerInput = controller;
    addRequirements(spinny); //"Locks the subsystem to the command"
  }

  @Override
    public void execute() {

    double resultForAngle = this.m_arm.getArmRotation();
    double resultForAmps = this.m_arm.getCurrentAmps();

    //First Checkpoint
    if (m_arm.isFrontSwitchActive()) {
      m_arm.setArmAngleMotorSpeed(0);
    } else if (m_arm.isBackSwitchActive()) {
      m_arm.setArmAngleMotorSpeed(0);
    }
 
    //Second Checkpoint
    //TODO CHECK IF DELAY IS NEGLIGIBLE OR NOT
    if (resultForAngle >= maxThreshold) {
      m_arm.setArmAngleMotorSpeed(0);
    } else if (resultForAngle <= minThreshold) {
      m_arm.setArmAngleMotorSpeed(0);
    }

    //Third Checkpoint
    if (resultForAmps >= 15) {
      m_arm.setArmAngleMotorSpeed(0);
    }

    //Joystick Input
    xBoxControllerInput.getLeftY(); //y is anglation
    //System.out.println(xBoxControllerInput.getLeftY());
    xBoxControllerInput.getRightX(); //x is extension
  }
}
