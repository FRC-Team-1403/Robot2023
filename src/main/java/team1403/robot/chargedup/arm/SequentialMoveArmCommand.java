package team1403.robot.chargedup.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Moves the arm to a given position while avoiding any obstancles.
 */
public class SequentialMoveArmCommand extends CommandBase{
  private final ArmSubsystem m_arm;

  private double m_intialPivotAngle;
  private double m_intialExtensionLength;
  private double m_initialWristAngle;
  private double m_initialIntakeSpeed;

  private final ArmState m_endState;


  public SequentialMoveArmCommand(ArmSubsystem arm, ArmState endState) {
    this.m_endState = endState;
    m_arm = arm;
  }

  @Override
  public void initialize() {
    this.m_intialPivotAngle = m_arm.getPivotAngleSetpoint();
    this.m_intialExtensionLength = m_arm.getExtensionLengthSetpoint();
    this.m_initialWristAngle = m_arm.getAbsoluteWristAngle();
    this.m_initialIntakeSpeed = m_arm.getIntakeSpeedSetpoint();
    super.initialize();
  }


  @Override
  public void execute() {
    if(Math.abs(m_endState.armPivot - m_intialPivotAngle) > 1) {
      m_arm.moveArm(m_initialWristAngle, m_initialIntakeSpeed, m_endState.armPivot, m_intialExtensionLength);
    } else {
      m_arm.moveArm(m_endState);
    }
    super.execute();
  }

  @Override
  public boolean isFinished() {
    if(Math.abs(m_endState.armPivot - m_intialPivotAngle) < 1) {
      return m_arm.isAtSetpoint();
    }
    return false;
  }
  
}
