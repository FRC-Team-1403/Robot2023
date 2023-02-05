package team1403.lib.device.test;

import team1403.lib.device.BaseDevice;
import team1403.lib.device.CurrentSensor;
import team1403.lib.device.Encoder;
import team1403.lib.device.MotorController;
import team1403.lib.util.CougarLogger;


/**
 * A fake MotorController for testing.
 */
@SuppressWarnings("PMD.DataClass")
public class FakeMotorController
       extends BaseDevice
       implements MotorController {
  /**
   * Constructor.
   *
   * @param name The name for this instance.
   * @param logger The logger to use for this instance.
   */
  public FakeMotorController(String name, CougarLogger logger) {
    this(name, logger, (Encoder)null, (CurrentSensor)null);
  }

  /**
   * Full Constructor.
   *
   * @param name The name for this instance.
   * @param logger The logger to use for this instance.
   * @param encoder The embedded encoder or null if none.
   * @param current The embedded current sensor or null if none.
   */
  public FakeMotorController(String name, CougarLogger logger,
                             Encoder encoder, CurrentSensor current) {
    super(name);
    m_logger = logger;
    m_encoder = encoder;
    m_currentSensor = current;
  }

  /**
   * Returns the logger this device is using.
   *
   * @return The logger bound by the constructor.
   */
  public final CougarLogger getCougarLogger() {
    return m_logger;
  }

  /**
   * Returns number of times {@link #stopMotor} was called.
   *
   * @return number of stopMotor() calls since constructed.
   */
  public final int countStopMotorCalls() {
    return m_stopCalls;
  }

  /**
   * Returns the last motor controller passed to {@link #follow}.
   *
   * @return null if follow() hasnt been called yet.
   */
  public MotorController getFollowing() {
    return m_follow;
  }

  /**
   * Returns last last voltage to {@link #setVoltage}.
   *
   * @return NaN if setVoltage not yet called.
   */
  public double getVoltage() {
    return m_voltage;
  }

  /**
   * Returns last last speed to {@link #setSpeed}.
   *
   * @return NaN if setSpeed not yet called.
   */
  public double getSpeed() {
    return m_speed;
  }

  @Override
  public void follow(MotorController source) {
    m_follow = source;
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }

  @Override
  public void setSpeed(double speed) {
    m_speed = speed;
  }

  @Override
  public void stopMotor() {
    m_speed = 0;
    ++m_stopCalls;
  }

  @Override
  public void setInverted(boolean isInverted) {
    m_inverted = isInverted;
  }

  @Override
  public boolean getInverted() {
    return m_inverted;
  }

  @Override
  public Encoder getEmbeddedEncoder() {
    if (m_encoder != null) {
      return m_encoder;
    }
    return MotorController.super.getEmbeddedEncoder();
  }

  @Override
  public CurrentSensor getEmbeddedCurrentSensor() {
    if (m_currentSensor != null) {
      return m_currentSensor;
    }
    return MotorController.super.getEmbeddedCurrentSensor();
  }

  private double m_speed = Double.NaN;
  private double m_voltage = Double.NaN;
  private int m_stopCalls;
  private MotorController m_follow;
  private boolean m_inverted;
  private final CougarLogger m_logger;
  private final Encoder m_encoder;
  private final CurrentSensor m_currentSensor;
}
