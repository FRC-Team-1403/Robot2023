package team1403.lib.device.wpi;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import team1403.lib.device.MotorController;
import team1403.lib.util.CougarLogger;

/**
 * Device implementation for a VictorSp motor controller.
 */
public class VictorSp extends VictorSP
                       implements MotorController {
  /**
   * Constructor.
   *
   * @param name The name for the device.
   * @param channel The CAN channel the motor is on.
   * @param logger The debug logger to use for the device.
   */
  public VictorSp(String name, int channel, CougarLogger logger) {
    super(channel);
    m_name = name;
    m_logger = logger;
  }

  /**
   * Return the WPI_VictorSP API so we can do something specific.
   *
   * @return The underlying {@code WPI_VictorSPX} instance.
   */
  public final VictorSP getVictorSpxApi() {
    return this;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  /**
   * Maybe create another motor controller class for something like this.
   */
  @Override
  public void follow(MotorController source) {
    m_logger.errorf("Unsupported follow: %s source: %s", getName(), source.getName());
  }

  @Override
  public final void setVoltageCompensation(double voltage) {
    m_logger.errorf("Unsupported setVoltageCompensation %s %f", voltage);
  }

  @Override
  public final void setSpeed(double speed) {
    m_logger.tracef("setSpeed %s %f", getName(), speed);
    super.set(speed);
  }
  
  @Override
  public void setPosition(double position) {
    m_logger.errorf("Unsupported SetPosition %s %f", getName(), position);
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
    m_logger.errorf("Unsupported setRampRate %s %f", getName(), rate);
  }

  @Override
  public void setAmpLimit(int limit) {
    m_logger.errorf("Unsupported setAmpLimit %s %d", getName(), limit);
  }

  private final CougarLogger m_logger;
  private final String m_name;
}
