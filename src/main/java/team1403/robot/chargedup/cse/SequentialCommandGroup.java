package team1403.robot.chargedup.cse;

import java.util.ArrayList;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs commands sequentially.
 */
public class SequentialCommandGroup extends CommandBase {
  public ArrayList<Command> commandsToRun;
  private int m_currIndex = 0;
  private int m_maxIndex = 0;
  private Command m_toRun;
  private boolean m_isFinished = false;
  private Pose2d m_startPose;
  private Consumer<Pose2d> m_onStart;

  /**
   * Constructor.
   */
  public SequentialCommandGroup(Pose2d startPose, Consumer<Pose2d> onStart, Command... c_iter) {
    this.commandsToRun = new ArrayList<>();
    for (Command i : c_iter) {
      this.commandsToRun.add(i);
    }
    this.m_maxIndex = this.commandsToRun.size();
    this.m_toRun = this.commandsToRun.get(this.m_currIndex);
    this.m_startPose = startPose;
    this.m_onStart = onStart;
  }

  public SequentialCommandGroup(Command... c_iter) {
    this(null, null, c_iter);
  }

  public void setM_startPose(Pose2d pose) {
    this.m_startPose = pose;
  }

  public void add(Command command) {
    this.commandsToRun.add(command);
    this.m_maxIndex++;
  }

  public void reset() {
    this.m_currIndex = 0;
    this.m_toRun = this.commandsToRun.get(this.m_currIndex);
    m_isFinished = false;
  }

  // Only called like this so you know to put it in the main teleop loop
  @Override
  public void schedule() {
    super.schedule();
    this.m_toRun.schedule();
    if (m_startPose != null) {
      m_onStart.accept(m_startPose);
    }
    reset();
  }

  @Override
  public void execute() {
    if (m_toRun.isFinished()) {
      m_toRun.end(false);
      m_currIndex++;
      if (m_currIndex == m_maxIndex) {
        m_isFinished = true;
        return;
      }
      m_toRun = commandsToRun.get(m_currIndex);
      m_toRun.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    reset();
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}