package team1403.lib.device.wpi;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import team1403.lib.device.CurrentSensor;
import team1403.lib.device.Encoder;
import team1403.lib.device.MotorController;
import team1403.lib.util.CougarLogger;

/**
 * Device implementation for a WPI_TalonSRX motor controller.
 */
public class TalonSrx extends WPI_TalonSRX
                      implements MotorController {
  /**
   * Constructor.
   *
   * @param name The name name for the device.
   * @param channel The CAN channel the motor is on.
   * @param logger The debug logger to use for the device.
   */
  public TalonSrx(String name, int channel, CougarLogger logger) {
    super(channel);
    m_name = name;
    m_logger = logger;
    m_encoder = new EmbeddedEncoder(name + ".Encoder");
    m_currentSensor = new EmbeddedCurrentSensor(name + ".CurrentSensor");
  }

  /**
   * Return the WPI_TalonSRX API so we can do something specific.
   *
   * @return The underlying {@code WPI_TalonSRX} instance.
   */
  public final WPI_TalonSRX getTalonSrxApi() {
    return this;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  /**
   * Follow another TalonSrx motor.
   *
   * @param source Must be a com.ctre.phoenix.motorcontrol.IMotorController
   *               (e.g. another TalonSrx or VictorSpx)
   *
   * @throws ClassCastException if motor is not compatible.
   */
  @Override
  public void follow(MotorController source) {
    m_logger.tracef("follow %s <- %s", getName(), source.getName());
    super.follow((IMotorController)source);  // Will throw an exception if source is not compatible.
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
  public boolean hasEmbeddedEncoder() {
    return true;
  }

  @Override
  public Encoder getEmbeddedEncoder() {
    return m_encoder;
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
     */
    public EmbeddedEncoder(String name) {
      m_encoderName = name;
    }

    @Override
    public final String getName() {
      return m_encoderName;
    }

    @Override
    public final double ticksPerRevolution() {
      return 4096;
    }

    @Override
    public final double getPositionTicks() {
      return getSelectedSensorPosition(1);
    }

    @Override
    public final double getRpm() {
      final double unitsPer100ms = getSelectedSensorVelocity(1);
      final double unitsPerMinute = unitsPer100ms * 10 * 60;
      return unitsPerMinute / 4096.0;
    }

    private final String m_encoderName;
  }

  /**
   * Implements the interface to the embedded current sensor.
   */
  private class EmbeddedCurrentSensor implements CurrentSensor {
    /**
     * Constructor.
     */
    public EmbeddedCurrentSensor(String name) {
      m_sensorName = name;
    }

    @Override
    public final String getName() {
      return m_sensorName;
    }

    @Override
    public final double getAmps() {
      return getStatorCurrent();
    }

    private final String m_sensorName;
  }

  private final EmbeddedEncoder m_encoder;
  private final EmbeddedCurrentSensor m_currentSensor;
  private final CougarLogger m_logger;
  private final String m_name;
}
