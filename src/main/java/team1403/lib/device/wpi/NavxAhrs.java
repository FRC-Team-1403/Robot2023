package team1403.lib.device.wpi;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import team1403.lib.device.GyroscopeDevice;



/**
 * An implementation of the gyroscope device for the AHRS Navx2.
 */
public class NavxAhrs implements GyroscopeDevice {

  private final AHRS m_ahrs;
  private final boolean m_inverted;
  private final String m_name;

  /**
   * Constructor for the AHRS Device connected through SPI.
   * https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
   * Documentation for the Ahrs device 
   *
   * @param name the name of this device
   *
   * @param port the SPI port this is plugged in on
   *
   * @param inverted whether or not the yaw is rotated the wrong way 
  */
  public NavxAhrs(String name, SPI.Port port, boolean inverted) {
    m_ahrs = new AHRS(port);
    m_inverted = inverted;
    m_name = name;
  }

  public final AHRS getAhrs(){
    return m_ahrs;
  }

  /**
  * Resets device.
  */
  @Override
  public void reset() {
    m_ahrs.reset();
  }

  /**
  * Gets angle of the device.
  *
  * @return angle of device
  */
  @Override
  public double getRawAngle() {
    return m_ahrs.getAngle() * (m_inverted ? -1 : 1);
  }

  /**
  * Gets angular velocity of the device.
  *
  * @return angular velocity of device
  */
  @Override
  public double getAngularVelocity() {
    return m_ahrs.getRawGyroY() * (m_inverted ? -1 : 1);
  }

  @Override
  public final String getName() {
    return m_name;
  }

  @Override
  public double getAngle() {
    return m_ahrs.getAngle();
  }

  @Override
  public void setAngleOffset(double angleOffset) {
    m_ahrs.setAngleAdjustment(angleOffset);
  }

  @Override
  public double getAngleOffset() {
    return m_ahrs.getAngleAdjustment();
  }


}
