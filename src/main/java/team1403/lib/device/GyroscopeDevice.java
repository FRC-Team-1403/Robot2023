package team1403.lib.device;

/**
* Interface that represents a gyroscope.
*/
public interface GyroscopeDevice extends Sensor {

  /**
  * resets gyro angle to 0.
  */
  public abstract void reset();

  /**
  * Gets the angle (in degrees) of the gyro.
  * Resets the X,Y,and Z angles
  *
  * @return the angle the gyroscope is at (relative to where it was when last reset)
  */
  public abstract double getRawAngle();

  /**
  *
  * @return the raw angle plus the configured offset
  */
  public double getAngle();

  /**
  * Gets the current angular velocity (about vertical axis) of the gyroscope.
  *
  * @return the current angular velocity (in degrees/sec)
  */
  public abstract double getAngularVelocity();

  /**
  * Configures the offset angle used by getAngle.
  */
  public void setAngleOffset(double angleOffset);

  /**
  * Gets the current angle offset.
  *
  * @return the current angle offset (in degrees)
  */
  public double getAngleOffset();

}

  
