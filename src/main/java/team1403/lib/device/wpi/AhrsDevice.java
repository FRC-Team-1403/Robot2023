package team1403.lib.device.wpi;

import team1403.lib.device.GyroscopeDevice;

import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;


/**
 * An implementation of the gyroscope device for the AHRS Navx2.
 */

public class AhrsDevice implements GyroscopeDevice {

  private final AHRS m_ahrs;
  private final boolean m_inverted;

  /**
   * Constructor for the AHRS Device connected through SPI.
   *
   * @param name the name of this device
   *
   * @param port the SPI port this is plugged in on
   *
   * @param inverted whether or not the yaw is rotated the wrong way 
   * (should be ccwl = +)
  */
  public AhrsDevice(String name, SPI.Port port, boolean inverted) {
    this.m_ahrs = new AHRS(port);
    this.m_inverted = inverted;
    m_name = name;
  }


  /**
  * Resets encoder.
  */
  @Override
  public void reset() {
    m_ahrs.zeroYaw();
  }

  /**
  * Gets angle of the encoder.
  *
  * @return angle of encoder
  */
  @Override
  public double getRawAngle() {
    return m_ahrs.getAngle() * (m_inverted ? -1 : 1);
  }

  /**
  * Gets angular velocity of the encoder.
  *
  * @return angular velocity of encoder
  */
  @Override
  public double getAngularVelocity() {
    return m_ahrs.getRawGyroY() * (m_inverted ? -1 : 1);
  }

  @Override
  public final String getName() {
    return m_name;
  }

  private final String m_name;

  @Override
  public double getAngle() {
    return m_ahrs.getAngle();
  }

  @Override
  public void set_angleOffset(double angleOffset) {
    m_ahrs.setAngleAdjustment(angleOffset);
    
  }

  @Override
  public double get_angleOffset() {
    return m_ahrs.getAngleAdjustment();
  }


}
