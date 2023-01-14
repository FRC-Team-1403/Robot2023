package team1403.lib.device.wpi;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import team1403.lib.device.CougarDoubleSolenoid;
import team1403.lib.util.CougarLogger;

/**
 * Device implementation for a WpiDoubleSolenoid.
 */
public class WpiDoubleSolenoid extends DoubleSolenoid
                               implements CougarDoubleSolenoid {

  /**
   * Constructor.
   *
   * @param name The name for the device.
   * @param forwardChannel The port number for the forward channel.
   * @param reverseChannel The port number for the reverse channel.
   * @param logger The debug logger to use for the device.
   */
  public WpiDoubleSolenoid(String name, int forwardChannel, 
                           int reverseChannel, CougarLogger logger) {
    super(PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
    m_name = name;
    m_logger = logger;
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public void setState(Value value) {
    m_logger.tracef("setState %s %s", getName(), getState().toString());
    super.set(value);
  }

  @Override
  public void toggle() {
    super.toggle();
    m_logger.tracef("toggled %s, new state is %s", getName(), getState().toString());
  }

  @Override
  public Value getState() {
    return super.get();
  }

  private final CougarLogger m_logger;
  private final String m_name;
}
