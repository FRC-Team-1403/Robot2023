package team1403.lib.device.wpi;

import edu.wpi.first.wpilibj.Compressor;

import team1403.lib.device.CougarCompressor;

/**
 *Class to make a real compressor device.
*/
public class WpiCompressor implements CougarCompressor {

  private final String m_name;
  private final Compressor m_compressor;
  private double m_maxPressure;
  private double m_minPressure;

  /**
   * Enum contains different ways to enable to compressor.
   */
  public enum Type {
    DIGITAL,
    ANALOG,
    HYBRID
  }

  /**
   * Constructor. 
   *
   * @param name
   *
   * @param compressor
   *
   */
  public WpiCompressor(String name, Compressor compressor) {
    this.m_name = name;
    this.m_compressor = compressor;
  }

  @Override
  public String getName() {
    return this.m_name;
  }

  @Override
  public void stop() {
    // disable the compressor 
    this.m_compressor.disable();
  }

  @Override
  public double getPressureSwitchValue() {
    return this.m_compressor.getPressure();
  }

  @Override
  public void setPressure(int maxVal, int minVal) {
    //Set a constant for the pressure of the compressor.
    this.m_maxPressure = maxVal;
    this.m_minPressure = minVal;

  }

  @Override
  public boolean isEnabled() {
    return this.m_compressor.isEnabled();
  }

  @Override
  public double getCurrent() {
    return this.m_compressor.getCurrent();
  }

  @Override
  public void start(Mode type) {
    // start the compressor
    switch (type) {
      case ANALOG:
        this.m_compressor.enableAnalog(m_minPressure, m_maxPressure);
        break;
      case HYBRID:
        this.m_compressor.enableHybrid(m_minPressure, m_maxPressure);
        break;
      case DIGITAL:
        this.m_compressor.enableDigital();
        break;

      default:
        break;
    }   
  }
}
