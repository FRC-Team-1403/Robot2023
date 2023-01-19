package team1403.lib.device.wpi;

import edu.wpi.first.wpilibj.Compressor;

import team1403.lib.device.CougarCompressor;

/**
 *Class to make a real compressor device.
*/
public class WpiCompressor extends Compressor implements CougarCompressor {

  /**
  * Enum contains different way to enable a compressor.
  */
  public enum m_Mode {
    ANALOG,
    DIGITAL,
    HYBRID
  }

  private final String m_name;
  private boolean m_maxPressure;
  private boolean m_minPressure;

  /**
   * Constructor. 
   *
   * @param name The name given to this device instance.
   * @param type The mode of the WPI compressor.
   *
   */
  public WpiCompressor(String name, m_Mode type) {
    super(type);
    this.m_name = name;
  }

  @Override
  public String getName() {
    return this.m_name;
  }

  @Override
  public void stop() {
    // disable the compressor 
    disable();
  }

  @Override
  public boolean getPressureSwitchValue() {
    return getPressureSwitchValue();
  }

  @Override
  public void setPressure(Boolean maxVal, Boolean minVal) {
    //Set a constant for the pressure of the compressor.
    this.m_maxPressure = maxVal;
    this.m_minPressure = minVal;

  }

  @Override
  public boolean isEnabled() {
    return this.m_compressor.isEnabled();
  }

  @Override
  public double getAmps() {
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
