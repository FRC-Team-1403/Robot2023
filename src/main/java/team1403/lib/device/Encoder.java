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
   * Get the rate of rotation in RPM.
   *
   * @return rate of encoder rotation
   */
  double getRpm();
}
