package team1403.lib.device;

/**
 * Represents an encoder.
 */
public interface Encoder extends Sensor {
  /**
   * Returns number of ticks per revolution within the encoder.
   *
   * @return value determined by underlying encoder.
   */
  double ticksPerRevolution();

  /**
   * Get the position of the encoder.
   *
   * @return The angle of a tick is determined by the encoder.
   *
   * @see ticksPerRevolution
   */
  double getPositionTicks();

  /**
   * Set conversion factor for position ticks.
   *
   * @param conversionFactor The conversion factor to be used with position.
   */
  void setPositionTickConversionFactor(double conversionFactor);

  /**
   * Set conversion factor for velocity ticks.
   *
   * @param conversionFactor The conversion factor to be used with velocity.
   */
  void setVelocityTickConversionFactor(double conversionFactor);

  /**
   * Get the rate of rotation in RPM.
   *
   * @return rate of encoder rotation
   */
  double getRpm();

  /**
   * Get the velocity of the motor. 
   * This returns the native units of 'RPM' by default, 
   * and can be changed by a scale factor using setVelocityConversionFactor().
   *
   * @return Number the RPM of the motor
   */
  double getVelocityTicks();

}
