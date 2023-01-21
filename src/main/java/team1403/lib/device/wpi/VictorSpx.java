package team1403.lib.device.wpi;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import team1403.lib.device.AdvancedMotorController;
import team1403.lib.util.CougarLogger;

/**
 * Device implementation for a WPI_VictorSPX motor controller.
 */
public class VictorSpx extends WPI_VictorSPX implements AdvancedMotorController {  
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
  public void follow(AdvancedMotorController source) {
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
    m_logger.errorf("Unsupported setPosition %s %f", getName(), position);
  }

  @Override 
  public void setPidGains(double p, double i, double d) {
    m_logger.errorf("Unsupported setPidGains %s %f %f %f", getName(), p, i, d);
  }

  @Override
  public void setIdleMode(CougarIdleMode mode) {
    m_logger.errorf("Unsupported setIdleMode %s %s", getName(), mode.toString());
  }

  @Override
  public void setRampRate(double rate) {
    configClosedloopRamp(rate);
  }
  
  @Override
  public void setAmpLimit(double amps) {
    m_logger.errorf("Unsupported setAmpLimit %s %d", getName(), amps);
  }
  
  private final CougarLogger m_logger;
  private final String m_name;
}
