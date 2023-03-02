package team1403.robot.chargedup.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.lib.util.ArmState;

public class SetpointArmCommand extends CommandBase {
  private final ArmState m_state;

  private final Arm_Subsystem m_arm;

  public SetpointArmCommand(Arm_Subsystem arm, ArmState state) {
    this.m_arm = arm;
    this.m_state = state;

    addRequirements(arm);
  }
  
  @Override
  public void execute() {
    m_arm.moveArm(m_state);
  }
}
