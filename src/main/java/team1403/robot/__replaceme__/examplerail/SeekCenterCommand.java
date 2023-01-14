package team1403.robot.__replaceme__.examplerail;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Seek to the center of the rail.
 */
public class SeekCenterCommand extends CommandBase {

  /**
   * Creates a new SeekCenterCommand.
   *
   * @param rail The rail subsystem this command is controlling.
   * @param toleranceTicks How many ticks is close enough.
   */
  public SeekCenterCommand(ExampleRail rail, double toleranceTicks) {
    setName("SeekRailCenter");
    addRequirements(rail);
    m_rail = rail;
    m_speed = rail.getRailConfig().motorSpeed;
    m_tolerance = toleranceTicks;
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
    // If we dont know the front yet, then go there.
    double front = m_rail.getFrontPositionTicks();
    if (Double.isNaN(front)) {
      executeFindFront();
      return;
    }

    double back = m_rail.getBackPositionTicks();
    if (Double.isNaN(back)) {
      executeFindBack();
      return;
    }

    // Go from our current position toward the middle
    // until we are in the tolerance zone.
    double middle = (front + back) / 2;
    double here = m_rail.getPositionTicks();
    if (here < middle - m_tolerance) {
      // Move toward back
      m_rail.setSpeed(m_speed);
      return;
    }

    if (here > middle + m_tolerance) {
      // Move toward front
      m_rail.setSpeed(-m_speed);
      return;
    }

    // stay here.
    m_rail.stop();
  }

  private void executeFindFront() {
    if (!m_rail.isAtFront()) {
      m_rail.setSpeed(-m_speed);
      return;
    }
    m_rail.stop();
    double ticks = m_rail.getPositionTicks();
    m_rail.setFrontPositionTicks(ticks);
  }

  private void executeFindBack() {
    if (!m_rail.isAtBack()) {
      m_rail.setSpeed(m_speed);
      return;
    }
    m_rail.stop();
    double ticks = m_rail.getPositionTicks();
    m_rail.setBackPositionTicks(ticks);
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
   */
  @Override
  public boolean isFinished() {
    double middle = (m_rail.getFrontPositionTicks()
                     + m_rail.getBackPositionTicks()) / 2;
    if (Double.isNaN(middle)) {
      return false;
    }
    double here = m_rail.getPositionTicks();
    return (here >= middle - m_tolerance) && (here <= middle + m_tolerance);
  }

  // The rail we're controlling.
  private final ExampleRail m_rail;

  // The speed we'll drive the motor with.
  private final double m_speed;

  // How many ticks is considered good enough.
  private final double m_tolerance;

}
