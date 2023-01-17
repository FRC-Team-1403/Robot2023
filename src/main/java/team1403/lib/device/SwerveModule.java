package team1403.lib.device;

/**
 * A Swerve Module controlls one wheel of a robot.
 * 
 * <p>A Swerve Module consists of a drive motor, steer motor, and an absolute
 * encoder. The drive and steer motors also have a relative encoder built in.
 */
public interface SwerveModule {

  /**
   * Configure the drive motor's relative encoder, the steer motor's relative
   * encoder, and the absolute encoder.
   * This includes setting drive and position conversion factors.
   */
  public void configEncoders();

  /**
   * Configure the drive motor. This includes inverting the motor, enabling
   * voltage compensation, setting the current limit, and the CAN frame period.
   */
  public void configDriveMotor();

  /**
   * Configure the drive motor. This includes inverting the motor, enabling
   * voltage compensation, setting the current limit, onboard PID values, and the
   * status frame period.
   */
  public void configSteerMotor();

  /**
   * Returns the angle of the steering motor between 0 and 2 pi.
   *
   * @return the angle of the steering motor.
   */
  public double getSteerAngle();

  /**
   * Returns the error between the steer motor angle vs the setpoint.
   *
   * @return the error
   */
  public double angleError(double setpoint);

  /**
   * Sets the speed and angle for the swerve module to drive at.
   */
  public void set(double driveVoltage, double steerAngle);

  /**
   * Gets the current angle reading of the encoder in radians.
   *
   * @return The current angle in radians. Range: [0, 2pi)
   */
  public double getAbsoluteEncoderAbsoluteAngle();
}
