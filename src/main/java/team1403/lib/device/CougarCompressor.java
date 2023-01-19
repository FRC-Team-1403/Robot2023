package team1403.lib.device;

/**
 *Interface for the compressor device. 
*/
public interface CougarCompressor extends Actuator {

  enum Mode {
    ANALOG,
    DIGITAL,
    HYBRID
  }

  /**
   * Starts the Compressor.
   *
   * @param type The type of mode.
   *
   */
  void start(Mode type);

  /**
   * Stops the Compressor.
   */
  void stop();

  /**
   * Gets the Pressure Switch Value.
   *
   * @return The pressure switch value
   *
   */
  boolean getPressureSwitchValue();

  /**
   * Sets the pressure value of the Compressor.
   *
   * @param maxVal The max pressure value.
   *
   *@param minVal The min pressure vakye
   */
  void  setPressure(Double maxVal, Double minVal);

  /**
   * Checks if the Compressor is enabled. 
   *
   * @return The status of the Compressor.
   *
   */
  boolean isEnabled();

  /**
   * Returns the current flowing through the compressor motor.
   *
   * @return the current value
   *
   */
  double getAmps();
}
