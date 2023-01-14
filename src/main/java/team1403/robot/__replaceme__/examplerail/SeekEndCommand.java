package team1403.robot.__replaceme__.examplerail;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Seek to an end position on the rail.
 */
public class SeekEndCommand extends CommandBase {
  /**
   * Specifies where on the rail to seek to.
   */
  public enum Position {
    /**
     * The beginning of the rail.
     */
    FRONT,

    /**
     * The end of the rail.
     */
    BACK
  }


  /**
   * Creates a new SeekEndCommand.
   *
   * @param rail The rail subsystem this command is controlling.
   * @param position The position to seek to
   */
  public SeekEndCommand(ExampleRail rail, Position position) {
    setName("SeekRail" + position);
    addRequirements(rail);
    m_rail = rail;
    m_goal = position;

    final double direction = position == Position.FRONT ? -1.0 : 1.0;
    m_speed = direction * rail.getRailConfig().motorSpeed;
  }

  /**
   * Returns the goal position we're seeking to.
   *
   * @return The goal bound by the constructor.
   */
  public final Position getGoal() {
    return m_goal;
  }

  /**
   * This command has no state so nothing to reset before it restarts.
   */
  @Override
  public void initialize() {
  }

  /**
   * Move the motor until toward the limit switch until it is triggered.
   */
  @Override
  public void execute() {
    m_rail.setSpeed(m_speed);
  }

  /**
   * Stop the rail from moving when the command finishes for whatever reason.
   */
  @Override
  public void end(boolean interrupted) {
    m_rail.stop();
  }

  
  /**
   * We are finished when we reach the desired position.
   *
   * <p>If we determined we are finished then also mark this
   * position as the terminal point for the rail. Note that
   * we might not have an encoder in which case we'll be setting
   * the position to NaN, but position would always be NaN anyway.
   */
  @Override
  public boolean isFinished() {
    if (m_goal == Position.FRONT) {
      boolean result = m_rail.isAtFront();
      if (result) {
        m_rail.setFrontPositionTicks(m_rail.getPositionTicks());
      }
      return result;
    }

    boolean result = m_rail.isAtBack();
    if (result) {
      m_rail.setBackPositionTicks(m_rail.getPositionTicks());
    }
    return result;

  }


  // The rail we're controlling.
  private final ExampleRail m_rail;

  // The goal for this command.
  private final Position m_goal;

  // The speed we'll drive the motor with.
  private final double m_speed;

}
