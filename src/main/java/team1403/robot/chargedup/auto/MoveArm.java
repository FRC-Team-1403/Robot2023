package team1403.robot.chargedup.auto;

import javax.xml.stream.events.StartDocument;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.chargedup.arm.ArmState;
import team1403.robot.chargedup.arm.ArmSubsystem;
import team1403.robot.chargedup.arm.SequentialMoveArmCommand;
import team1403.robot.chargedup.arm.SetpointArmCommand;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

public class MoveArm extends CommandBase{
  private final ArmSubsystem m_arm;
  private final SwerveSubsystem m_swerve;

  private final Pose2d m_startingPosition;
  private final ArmState m_desiredArmState;

  private final double m_distanceFromScoringLocation;

  private SequentialMoveArmCommand sequentialCommand;
  private SetpointArmCommand setpointCommand;

  public MoveArm(ArmSubsystem arm, SwerveSubsystem swerve, Pose2d startingPosition, ArmState desiredArmState) {
    m_arm = arm;
    m_swerve = swerve;
    m_startingPosition = startingPosition;
    m_desiredArmState = desiredArmState;

    m_distanceFromScoringLocation = m_startingPosition.getTranslation().getX() - 2;
  }

  @Override
  public void initialize() {
    if(m_distanceFromScoringLocation < 1) {
      sequentialCommand = new SequentialMoveArmCommand(m_arm, m_desiredArmState);
      sequentialCommand.schedule();
    } else {
      setpointCommand = new SetpointArmCommand(m_arm, m_desiredArmState);
      setpointCommand.schedule();
    }
    super.initialize();
  }

  @Override
  public boolean isFinished() {
    if(sequentialCommand != null) {
      return sequentialCommand.isFinished();
    } else {
      return setpointCommand.isFinished();
    }
  }
}
