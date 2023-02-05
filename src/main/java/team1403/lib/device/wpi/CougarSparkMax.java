package team1403.lib.device.wpi;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import team1403.lib.device.CurrentSensor;
import team1403.lib.device.Encoder;
import team1403.lib.device.MotorController;
import team1403.lib.device.NoSuchDeviceError;
import team1403.lib.util.CougarLogger;

/**
 * Device implementation for a base CANSparkMax motor controller.
 */
public final class CougarSparkMax extends CANSparkMax 
                            implements MotorController {

  /**
   * Method for creating brushless CANSparkMax.
   *
   * @param name The name name for the device.
   * @param channel The CAN channel the motor is on.
   * @param encoderType the type of encoder attached
   * @param logger The debug logger to use for the device.
   * @return Brushless CANSparkMax
   */
  public static CougarSparkMax makeBrushless(String name, int channel,
                               SparkMaxRelativeEncoder.Type encoderType, CougarLogger logger) {
    return new CougarSparkMax(name, channel, MotorType.kBrushless, encoderType, logger);
  }
  
  /**
   * Method for creating brushed CANSparkMax.
   *
   * @param name The name name for the device.
   * @param channel The CAN channel the motor is on.
   * @param encoderType the type of encoder attached
   * @param logger The debug logger to use for the device.
   * @return Brushed CANSparkMax
   */
  public static CougarSparkMax makeBrushed(String name, int channel,
                               SparkMaxRelativeEncoder.Type encoderType, CougarLogger logger) {
    return new CougarSparkMax(name, channel, MotorType.kBrushed, encoderType, logger);
  }

  /**
   * Constructor.
   *
   * @param name The name name for the device.
   * @param channel The CAN channel the motor is on.
   * @param motorType The type of motor connected 
   * @param encoderType the type of encoder attached
   * @param logger The debug logger to use for the device.
   */
  private CougarSparkMax(String name, int channel, MotorType motorType,
                         SparkMaxRelativeEncoder.Type encoderType, CougarLogger logger) {
    super(channel, motorType);
    m_name = name;
    m_logger = logger;
    m_encoder = encoderType != SparkMaxRelativeEncoder.Type.kNoSensor 
      ? new EmbeddedEncoder(name + ".Encoder", getEncoder(encoderType, 4096)) : null; 
    m_currentSensor = new EmbeddedCurrentSensor(name + ".CurrentSensor");
  }

  /**
   * Return the CANSparkMax API so we can do something specific.
   *
   * @return The underlying {@code CANSparkMax} instance.
   */
  public final CANSparkMax getCanSparkMaxApi() {
    return this;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  /**
   * Follow another CANSParkMax motor.
   *
   * @param source Must be a com.revrobotics.CANSparkMax
   *
   * @throws ClassCastException if motor is not compatible.
   */
  @Override
  public void follow(MotorController source) {
    m_logger.tracef("follow %s <- %s", getName(), source.getName());
    follow((CANSparkMax)source);  // Will throw an exception if source is not compatible.
  }

  @Override
  public final void set(double speed) {
    setSpeed(speed);
  }

  @Override
  public final void setSpeed(double speed) {
    m_logger.tracef("setSpeed %s %f", getName(), speed);
    super.set(speed);
  }

  @Override
  public final void stopMotor() {
    m_logger.tracef(("stopMotor %s"), getName());
    super.stopMotor();
  }

  @Override
  public boolean hasEmbeddedEncoder() {
    return m_encoder != null;
  }

  @Override
  public Encoder getEmbeddedEncoder() {
    if (hasEmbeddedEncoder()) {
      return m_encoder;
    }
    throw new NoSuchDeviceError("No Encoder with " + m_name);
  }

  @Override
  public boolean hasEmbeddedCurrentSensor() {
    return true;
  }

  @Override
  public CurrentSensor getEmbeddedCurrentSensor() {
    return m_currentSensor;
  }
  /**
   * Implements the interface to the embedded encoder.
   */

  private class EmbeddedEncoder implements Encoder {
    /**
     * Constructor.

     * @param name The name of encoder
     * @param encoderType the type of encoder used
     */
    public EmbeddedEncoder(String name, RelativeEncoder encoder) {
      m_encoderName =  name;
      m_encoder = encoder;
    }

    @Override
    public final String getName() {
      return m_encoderName;
    }

    @Override
    public final double ticksPerRevolution() {
      return m_encoder.getCountsPerRevolution();
    }

    @Override
    public final double getPositionTicks() {
      return m_encoder.getPosition();
    }

    @Override
    public final double getRpm() {
      return m_encoder.getVelocity();
    }

    private final String m_encoderName;
    private final RelativeEncoder m_encoder;
  }

  /**
   * Implements the interface to the embedded current sensor.
   *
   * <p>This is not a static class so instances share the
   * CougarSparkMax instance state.
   */
  private class EmbeddedCurrentSensor implements CurrentSensor {
    /**
     * Constructor.
     */
    public EmbeddedCurrentSensor(String name) {
      m_sensorName =  name;
    }

    @Override
    public final String getName() {
      return m_sensorName;
    }

    @Override
    public final double getAmps() {
      return getOutputCurrent();
    }

    private final String m_sensorName;
  }

  private final EmbeddedEncoder m_encoder;
  private final EmbeddedCurrentSensor m_currentSensor;
  private final CougarLogger m_logger;
  private final String m_name;
}
