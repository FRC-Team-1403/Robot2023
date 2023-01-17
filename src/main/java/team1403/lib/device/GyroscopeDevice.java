package team1403.lib.device;

/**
* Interphase that represents a gyroscope.
*/
public interface GyroscopeDevice extends Sensor {



  /**
  * resets gyro.
  */
  public abstract void reset();

  /**
  * Gets the angle (in degrees) of the gyro.
  *
  * @return the angle the gyroscope is at (relative to where it started)
  */
  public abstract double getRawAngle();

  /**
  * Returns the raw angle offsetted by some amount. 
  * The offset is just a basic "add" to the angle,
  * so it moves 0 to that point.
  *
  * @return the angle offset by some amount
*/
  public double getAngle();

  /**
  * Gets the current angular velocity (about vertical axis) of the gyroscope.
  *
  * @return the current angular velocity (int degrees/sec)
  */
  public abstract double getAngularVelocity();

  /**
  * Sets the angle offset to the inputed value.
  */
  public void set_angleOffset(double angleOffset);

  /**
  * Gets the current angle offset.
  *
  * @return the current angle offset (in degrees)
  */
  public double get_angleOffset();

}

  
