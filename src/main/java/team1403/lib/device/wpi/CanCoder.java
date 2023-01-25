package team1403.lib.device.wpi;

import com.ctre.phoenix.sensors.CANCoder;

import team1403.lib.device.Encoder;


/**
 * Implements the encoder.
 */
public class CanCoder extends CANCoder implements Encoder {

  private final double m_ticksPerRevolution;
  private double m_positionConversionFactor = 1.0;
  private double m_velocityConversionFactor = 1.0;
  private final String m_name;

  /**
   * Constructor.
   *
   * @param name The name of the instance.
   * @param ticksPerRevolution The ticks per revolution is fixed.
   */
  public CanCoder(String name, double ticksPerRevolution, int portNumber) {
    super(portNumber);
    m_name = name;
    m_ticksPerRevolution = ticksPerRevolution;
  }

  @Override
  public double getPositionTicks() {
    return super.getPosition() * m_positionConversionFactor;
  }

  @Override
  public double getRpm() {
    return super.getVelocity() * m_velocityConversionFactor;
  }

  @Override
  public double ticksPerRevolution() {
    return m_ticksPerRevolution;
  }

  @Override
  public void setPositionTickConversionFactor(double conversionFactor) {
    m_positionConversionFactor = conversionFactor;
  }

  @Override
  public void setVelocityTickConversionFactor(double conversionFactor) {
    m_velocityConversionFactor = conversionFactor;
  }

  @Override
  public String getName() {
    return m_name;
  }
}
