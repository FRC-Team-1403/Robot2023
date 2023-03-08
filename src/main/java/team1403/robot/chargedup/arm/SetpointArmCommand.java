package team1403.robot.chargedup.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * Creates the arm set point class.
 */
public class SetpointArmCommand extends CommandBase {
  private final ArmState m_state;

  private final ArmSubsystem m_arm;
  
  /**
   * Initializes the class.
   */
  public SetpointArmCommand(ArmSubsystem arm, ArmState state) {
    this.m_arm = arm;
    this.m_state = state;

    addRequirements(arm);
  }
  
  @Override
  public void initialize() {
    m_arm.moveArm(m_state);
  }

  @Override
  public boolean isFinished() {
    return m_arm.isAtSetpoint();
  }
}
