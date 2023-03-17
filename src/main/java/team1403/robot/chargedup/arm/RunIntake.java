package team1403.robot.chargedup.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the intake while perserving the rest of the arm subsystem's setpoints.
 */
public class RunIntake extends CommandBase{
  private double m_intialPivotAngle;
  private double m_intialExtensionLength;
  private double m_initialWristAngle;

  private double startTime;

  private double m_intakeSpeed;

  private ArmSubsystem m_arm;

  public RunIntake(ArmSubsystem arm, double intakeSpeed) {
    this.m_arm = arm;
    this.m_intakeSpeed = intakeSpeed;
  }

  @Override
  public void initialize() {
    this.m_intialPivotAngle = m_arm.getPivotAngleSetpoint();
    this.m_intialExtensionLength = m_arm.getExtensionLengthSetpoint();
    this.m_initialWristAngle = m_arm.getAbsoluteWristAngle();
    this.startTime = Timer.getFPGATimestamp();

    m_arm.moveArm(new ArmState(m_intialExtensionLength, m_initialWristAngle, m_intialPivotAngle, m_intakeSpeed));
  }


  @Override
  public boolean isFinished() {
    System.out.println(Timer.getFPGATimestamp() - startTime);
    return (Timer.getFPGATimestamp() - startTime) >= 0.5;
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.moveArm(new ArmState(m_intialExtensionLength, m_initialWristAngle, m_intialPivotAngle, 0));
  }
}
