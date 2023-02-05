package team1403.lib.device.test;

import team1403.lib.device.BaseDevice;
import team1403.lib.device.Encoder;


/**
 * Implements a fake encoder.
 */
public class FakeEncoder extends BaseDevice implements Encoder {
  /**
   * Constructor.
   *
   * @param name The name of the instance.
   * @param ticksPerRevolution The fake's ticks per revolution is fixed.
   */
  public FakeEncoder(String name, double ticksPerRevolution) {
    super(name);
    m_ticksPerRevolution = ticksPerRevolution;
  }

  /**
   * Sets the current RPM to return.
   *
   * @param rpm The revolutions per minute.
   */
  public void setRpm(double rpm) {
    m_rpm = rpm;
  }

  /**
   * Set the current positional ticks to return.
   *
   * @param ticks The number of ticks for the current position.
   */
  public void setPositionTicks(double ticks) {
    m_ticks = ticks;
  }

  /**
   * Sets the current positional ticks in terms of revolutions.
   *
   * @param revs The number of revolutions implies positional ticks.
   */
  public void setRevolutions(double revs) {
    m_ticks = revs * m_ticksPerRevolution;
  }

  @Override
  public double getPositionTicks() {
    return m_ticks;
  }

  @Override
  public double getRpm() {
    return m_rpm;
  }

  @Override
  public double ticksPerRevolution() {
    return m_ticksPerRevolution;
  }

  private final double m_ticksPerRevolution;
  private double m_ticks;
  private double m_rpm;
}
