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

  /**
  * Contructor.
  *
  *@param name
  *
  *@param current
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
  public double getPressureSwitchValue() {
    return this.m_maxPressure;
  }

  @Override
  public void setPressure(int maxVal, int minVal) {
    //Set a constant for the pressure of the compressor.
    this.m_maxPressure = maxVal;

  }

  @Override
  public boolean isEnabled() {
    return this.m_state;
  }

  @Override
  public double getCurrent() {
    return this.m_current;
  }

  @Override
  public void start(Mode type) {
    // start the device 
    this.m_state = true;

  }

}
