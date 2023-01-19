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
  private double m_maxPressure;
  private double m_minPressure;

  /**
   * Constructor. 
   *
   * @param name The name given to this device instance.
   * @param type The mode of the WPI compressor.
   *
   */
  public WpiCompressor(String name){ 
    super(null);
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
  public void setPressure(Double maxVal, Double minVal) {
    //Set a constant for the pressure of the compressor.
    this.m_maxPressure = maxVal;
    this.m_minPressure = minVal;


  }

  @Override
  public boolean isEnabled() {
    return this.isEnabled();
  }

  @Override
  public double getAmps() {
    return this.getCurrent();
  }

  @Override
  public void start(Mode type) {
    // start the compressor
    switch (type) {
      case ANALOG:
        this.enableAnalog(m_minPressure, m_maxPressure);
        break;
      case HYBRID:
        this.enableHybrid(m_minPressure, m_maxPressure);
        break;
      case DIGITAL:
        this.enableDigital();
        break;

      default:
        break;
    }   
  }

}
