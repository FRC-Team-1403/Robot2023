package team1403.robot.chargedup.cse;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs multiple commands simulataneously.
 */
public class ParallelCommand extends CommandBase {

  private final ArrayList<Command> m_commandsToRun;
  private final ArrayList<Command> m_endCommands;

  public ParallelCommand(ArrayList<Command> commandsToRun, ArrayList<Command> endCommands) {
    this.m_commandsToRun = commandsToRun;
    this.m_endCommands = endCommands;
  } 

  @Override
  public void initialize() {
    for (Command i : m_commandsToRun) {
      i.initialize();
    }
  }

  @Override
  public void execute() {
    for (Command i : m_commandsToRun) {
      System.out.println(i.getClass());
      i.execute();
    }
  }

  @Override
  public void schedule() {
    for (Command c : m_commandsToRun) {
      c.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    for (Command c : m_commandsToRun) {
      c.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    for (Command i : m_endCommands) {
      if (!i.isFinished()) {
        return false;
      }
    }
    return true;
  }
}