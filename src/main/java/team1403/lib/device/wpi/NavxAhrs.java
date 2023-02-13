package team1403.lib.device.wpi;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

import team1403.lib.device.GyroscopeDevice;

/**
 * The NavX implementation of the gyroscope device. Assumes the NavX is
 * connected to the MXP port.
 */
public class NavxAhrs extends AHRS implements GyroscopeDevice{
  private final String m_name;
  private final AHRS m_navx;

  public NavxAhrs(String name) {
    this.m_name = name;
    this.m_navx = new AHRS(SPI.Port.kMXP);
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public void reset() {
    m_navx.reset();
  }

  @Override
  public double getRawAngle() {
    return m_navx.getAngle() - m_navx.getAngleAdjustment();
  }

  @Override
  public double getAngle() {
    return m_navx.getAngle();
  }

  @Override
  public double getAngularVelocity() {
    return m_navx.getRate();
  }

  @Override
  public void setAngleOffset(double angleOffset) {
    m_navx.setAngleAdjustment(angleOffset);
  }

  @Override
  public double getAngleOffset() {
    return m_navx.getAngleAdjustment();
  }

  @Override
  public Rotation2d getRotation2d() {
    return super.getRotation2d();
  }

}