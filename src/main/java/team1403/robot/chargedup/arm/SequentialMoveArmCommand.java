package team1403.robot.chargedup.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private TrapezoidProfile m_pivotProfile;

  private double m_startTime;

  private int currentState;
  private ArmState[] armStates = new ArmState[2];


  public SequentialMoveArmCommand(ArmSubsystem arm, ArmState endState) {
    this.m_endState = endState;
    this.m_arm = arm;
    this.currentState = 0;

  }

  @Override
  public void initialize() {
    this.m_intialPivotAngle = m_arm.getPivotAngleSetpoint();
    this.m_intialExtensionLength = m_arm.getExtensionLengthSetpoint();
    this.m_initialWristAngle = m_arm.getAbsoluteWristAngle();
    this.m_initialIntakeSpeed = m_arm.getIntakeSpeedSetpoint();
    
    this.m_pivotProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(20, 10), //high --> 360, 165 //slow --> 20, 10
      new TrapezoidProfile.State(m_endState.armPivot, 1),
      new TrapezoidProfile.State(m_intialPivotAngle, 0));

    this.m_startTime = Timer.getFPGATimestamp();

    this.armStates[0] = new ArmState(m_intialExtensionLength , m_initialWristAngle, m_endState.armPivot, m_initialIntakeSpeed);
    this.armStates[1] = this.m_endState;
    System.out.println("End arm state: " + armStates[1]);

    super.initialize();
  }


  @Override
  public void execute() {
    if(currentState == 0) {
      double deltaT = Timer.getFPGATimestamp() - m_startTime;
      double pivotPosition = m_pivotProfile.calculate(deltaT).position;
      ArmState state = new ArmState(this.armStates[currentState].armLength, this.armStates[currentState].wristAngle, pivotPosition, this.armStates[currentState].intakeSpeed);
      m_arm.moveArm(state);
    } else {
      m_arm.moveArm(this.armStates[currentState]);
    }


    if(m_arm.isAtSetpoint()) {
      System.out.println("____________________________Changing_______________");
      currentState++;
    }
    super.execute();
  }

  @Override
  public boolean isFinished() {
    return currentState == 2;
  }
  
}
