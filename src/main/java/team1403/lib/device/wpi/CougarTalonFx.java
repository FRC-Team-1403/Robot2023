package team1403.lib.device.wpi;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import team1403.lib.device.CurrentSensor;
import team1403.lib.device.Encoder;
import team1403.lib.device.MotorController;
import team1403.lib.device.NoSuchDeviceError;
import team1403.lib.util.CougarLogger;

public class CougarTalonFx extends TalonFX implements MotorController {

  private final EmbeddedEncoder m_encoder;
  private final EmbeddedCurrentSensor m_currentSensor;
  TalonFXControlMode controlMode;
  String m_name;
  CougarLogger m_logger;

  public CougarTalonFx(String name, int deviceNumber, TalonFXControlMode controlMode, CougarLogger m_logger) {
    super(deviceNumber);
    this.controlMode = controlMode;
    this.m_logger = m_logger;
    m_name = name;
    m_encoder = new EmbeddedEncoder(name + ".Encoder");
    m_currentSensor = new EmbeddedCurrentSensor(name + ".CurrentSensor");
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public void follow(MotorController source) {
    m_logger.tracef("follow %s <- %s", getName(), source.getName());
    super.follow((TalonFX) source);

  }

  @Override
  public void setSpeed(double speed) {
    throw new UnsupportedOperationException();
  }

  @Override
  public void setPosition(double position) {
    m_logger.tracef("setPosition %s %f", getName(), position);
    set(controlMode, position);
  }

  @Override
  public void stopMotor() {
    m_logger.tracef(("stopMotor %s"), getName());
    set(TalonFXControlMode.Velocity, 0);
  }

  @Override
  public final void setVoltageCompensation(double voltage) {
    m_logger.tracef("setVoltage %s %f", getName(), voltage);
    super.configVoltageCompSaturation(voltage);
  }

  @Override
  public void setRampRate(double rate) {
    super.configClosedloopRamp(rate);
    
  }

  @Override
  public void setCurrentLimit(int limit) {
    super.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, limit, 0, 0));
  }

  @Override 
  public void setGains(double p, double i, double d) {
    SlotConfiguration config = new SlotConfiguration();
    config.kP = p;
    config.kD = d;
    config.kI = i;
    super.configureSlot(config, 0, getBaseID())
  }


  @Override
  public boolean hasEmbeddedEncoder() {
    return m_encoder != null;
  }

  @Override
  public Encoder getEmbeddedEncoder() {
    if (hasEmbeddedEncoder()) {
      return m_encoder;
    }
    throw new NoSuchDeviceError("No Encoder with " + m_name);
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

    @Override
    public void setPositionTickConversionFactor(double conversionFactor) {
      // TODO Auto-generated method stub
      
    }

    @Override
    public void setVelocityTickConversionFactor(double conversionFactor) {
      // TODO Auto-generated method stub
      
    }
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

}
