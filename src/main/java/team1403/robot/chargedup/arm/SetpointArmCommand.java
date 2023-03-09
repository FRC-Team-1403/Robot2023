package team1403.robot.chargedup.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * Creates the arm set point class.
 */
public class SetpointArmCommand extends CommandBase {
  private final ArmState m_state;
  private TrapezoidProfile m_pivotProfile;
  private final ArmSubsystem m_arm;

  private double m_startTime;
  
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
    this.m_pivotProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(360, 165), //360, 165
      new TrapezoidProfile.State(m_state.armPivot, 1),
      new TrapezoidProfile.State(m_arm.getAbsolutePivotAngle(), 0));
    this.m_startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double deltaT = Timer.getFPGATimestamp() - m_startTime;
    double pivotPosition = m_pivotProfile.calculate(deltaT).position;
    SmartDashboard.putNumber("Pivot Auto Setpoint", pivotPosition);
    m_arm.moveArm(m_state.wristAngle, m_state.intakeSpeed, pivotPosition, m_state.armLength);
    super.execute();
  }

  @Override
  public boolean isFinished() {
    return m_pivotProfile.isFinished(Timer.getFPGATimestamp() - m_startTime);
  }
}
