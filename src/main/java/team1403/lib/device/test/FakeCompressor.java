package team1403.lib.device.test;

import team1403.lib.device.CougarCompressor;

/**
 * Class to make a fake compressor device.
 */
public class FakeCompressor implements CougarCompressor {
  private final String m_name;
  private double m_maxPressure;
  private boolean m_state;
  private final double m_current;
  private double max_pressure = 0;
  private double min_pressure = 0;

  /**
   * Contructor.
   *
   * @param name
   *
   * @param current
   *
   */
  public FakeCompressor(String name, double current) {
    this.m_name = name;
    this.m_current = current;

  }

  @Override
  public String getName() {
    return this.m_name;
  }

  @Override
  public void stop() {
    // disable the compressor
    this.m_state = false;

  }

  @Override
  public double getAmps() {
    return this.m_current;
  }

  @Override
  public void start(Mode type) {
    // start the device
    this.m_state = true;

  }

  @Override
  public boolean getPressureSwitchValueState() {
    return m_state;
  }

  @Override
  public void setPressure(Double maxVal, Double minVal) {
    max_pressure = maxVal;
    min_pressure = minVal;
  }

  @Override
  public boolean enabled() {
    return m_state;
  }
}
