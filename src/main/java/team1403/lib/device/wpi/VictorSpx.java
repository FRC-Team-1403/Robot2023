package team1403.lib.device.wpi;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import team1403.lib.device.MotorController;
import team1403.lib.util.CougarLogger;

/**
 * Device implementation for a WPI_VictorSPX motor controller.
 */
public class VictorSpx extends WPI_VictorSPX implements MotorController {  
  /**
   * Constructor.
   *
   * @param name The name name for the device.
   * @param channel The CAN channel the motor is on.
   * @param logger The debug logger to use for the device.
   */
  public VictorSpx(String name, int channel, CougarLogger logger) {
    super(channel);
    m_name = name;
    m_logger = logger;
  }

  /**
   * Return the WPI_VictorSPX API so we can do something specific.
   *
   * @return The underlying {@code WPI_VictorSPX} instance.
   */
  public final WPI_VictorSPX getVictorSpxApi() {
    return this;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  /**
   * Follow another VictorSpx motor.
   *
   * @param source Must be a com.ctre.phoenix.motorcontrol.IMotorController
   *               (e.g. another VictorSpx or TalonSpx)
   *
   * @throws ClassCastException if motor is not compatible.
   */
  @Override
  public void follow(MotorController source) {
    m_logger.tracef("follow %s <- %s", getName(), source.getName());
    super.follow((IMotorController)source);  // Will throw an exception if source is not compatible.
  }

  @Override
  public void setVoltageCompensation(double voltage) {
    super.configVoltageCompSaturation(voltage);
  }

  @Override
  public final void setSpeed(double speed) {
    m_logger.tracef("setSpeed %s %f", getName(), speed);
    super.set(speed);
  }

  @Override
  public void setPosition(double position) {
    throw new UnsupportedOperationException();
  }

  @Override
  public void setInverted(boolean isInverted) {
    super.setInverted(isInverted);
  }

  @Override
  public boolean getInverted() {
    return super.getInverted();
  }

  @Override 
  public void setGains(double p, double i, double d) {
    throw new UnsupportedOperationException();
  }

  @Override
  public void setIdleMode(CougarIdleMode mode) {
    throw new UnsupportedOperationException();
  }

  @Override
  public void setRampRate(double rate) {
    configClosedloopRamp(rate);
  }

  @Override
  public void stopMotor() {
    super.stopMotor();
  }

  @Override
  public void setCurrentLimit(int limit) {
    throw new UnsupportedOperationException();
  }
  
  private final CougarLogger m_logger;
  private final String m_name;
}
