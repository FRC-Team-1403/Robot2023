package team1403.robot.__replaceme__.examplerail;

import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.CurrentSensor;
import team1403.lib.device.Encoder;
import team1403.lib.device.LimitSwitch;
import team1403.lib.device.MotorController;
import team1403.lib.device.virtual.LimitSwitchImpl;
import team1403.robot.__replaceme__.RobotConfig;

/**
 * A placeholder subsystem for purposes of writing the skeleton RobotContainer.
 */
public class ExampleRail extends CougarSubsystem {
  /** 
   * Constructor.
   *
   * @param injectedParameters The standard CougarRobot injected parameters.
   * @param robotConfig Provides the ExampleRail configuration for tuning.
   */
  public ExampleRail(CougarLibInjectedParameters injectedParameters,
                     RobotConfig robotConfig) {
    super("Rail", injectedParameters);

    m_railConfig = robotConfig.exampleRail;
    var can = robotConfig.canBus;
    var ports = robotConfig.ports;

    // We'll use this same logger for the devices.
    var logger = getLogger();

    var factory = injectedParameters.getDeviceFactory();
    if (can.exampleRailSparkMotor >= 0) {
      m_motor = factory.makeBrushlessCanSparkMax(
          "Rail.Motor", can.exampleRailSparkMotor,
          SparkMaxRelativeEncoder.Type.kQuadrature,
          logger);
    } else {
      m_motor = factory.makeTalon("Rail.Motor", can.exampleRailMotor, logger);
    }
    m_motor.setInverted(m_railConfig.motorInverted);
    m_frontLimitSwitch
        = factory.makeLimitSwitch("Rail.Front",
                                  ports.exampleRailForwardLimitSwitch);
    if (ports.exampleRailReverseLimitSwitch < 0) {
      // We dont have a real switch so we'll create a virtual one
      // based on some predermined rail length. It will trigger
      // when the encoder is that distance from the front encoder.
      // It is undefined until the front encoder is determined.
      final long railLengthInTicks = m_railConfig.virtualBackLimitSwitchTicks;

      getLogger().debugf("Using virtual limit switch for back at ticks=%d",
                         railLengthInTicks);
      m_backLimitSwitch = new LimitSwitchImpl("Rail.Back", () -> {
        if (Double.isNaN(m_frontPositionTicks)) {
          return false;
        }
        var backPosition = m_frontPositionTicks + railLengthInTicks;
        return getPositionTicks() >= backPosition;
      });
    } else {
      m_backLimitSwitch
        = factory.makeLimitSwitch("Rail.Back",
                                  ports.exampleRailReverseLimitSwitch);
    }

    if (m_motor.hasEmbeddedEncoder()) {
      m_encoder = m_motor.getEmbeddedEncoder();
    } else {
      m_encoder = null;
    }

    if (m_motor.hasEmbeddedCurrentSensor()) {
      m_currentSensor = m_motor.getEmbeddedCurrentSensor();
    } else {
      m_currentSensor = null;
    }
  }

  /**
   * Return rail configuration.
   *
   * @return The configuration bound in the constructor.
   */
  public RobotConfig.ExampleRail getRailConfig() {
    return m_railConfig;
  }

  /**
   * Stop the rail motor.
   */
  public void stop() {
    tracef("rail.stop");
    m_motor.setSpeed(0.0);
    m_motor.stopMotor();
    SmartDashboard.putNumber("Rail.Speed", 0.0);
  }

  /**
   * Move the rail motor at the given speed.
   *
   * @param speed -1.0..1.0 of motor power.
   *              Positive is forward, negative is backward.
   */
  public void setSpeed(double speed) {
    tracef("rail.setSpeed %f", speed);

    if (speed > -m_railConfig.minSpeed && speed < m_railConfig.minSpeed) {
      stop();
      return;
    }

    m_motor.setSpeed(speed);
    SmartDashboard.putNumber("Rail.Speed", speed);
  }

  /**
   * Determine if motor is at the very front of the rail.
   *
   * @return true if front limit switch is triggered, otherwise false.
   */
  public boolean isAtFront() {
    return m_frontLimitSwitch.isTriggered();
  }

  /**
   * Determine if motor is at the very end of the rail.
   *
   * @return true if back limit switch is triggered, otherwise false.
   */
  public boolean isAtBack() {
    return m_backLimitSwitch.isTriggered();
  }

  /**
   * Return the current position of the rail motor.
   *
   * @return The position or NaN if there is no encoder.
   */
  public double getPositionTicks() {
    if (m_encoder == null) {
      getLogger().debugf("getRailPositionTicks called without encoder.");
      return Double.NaN;
    }
    return m_encoder.getPositionTicks();
  }

  /**
   * Return the current position of the front limit.
   *
   * @return The position or NaN if not yet known.
   *
   * @see setFrontPositionTicks
   */
  public double getFrontPositionTicks() {
    return m_frontPositionTicks;
  }

  /**
   * Sets the position of the front limit switch.
   *
   * @param ticks The position to set for.
   */
  public void setFrontPositionTicks(double ticks) {
    getLogger().debugf("setFrontPositionTicks %f", ticks);
    m_frontPositionTicks = ticks;
  }

  /**
   * Return the current position of the back limit.
   *
   * @return The position or NaN if not yet known.
   *
   * @see setBackPositionTicks
   */
  public double getBackPositionTicks() {
    return m_backPositionTicks;
  }

  /**
   * Sets the position of the front limit switch.
   *
   * @param ticks The position to set for.
   */
  public void setBackPositionTicks(double ticks) {
    getLogger().debugf("setFrontPositionTicks %f", ticks);
    m_backPositionTicks = ticks;
  }

  /**
   * Write some metrics to the smart dashboard.
   */
  public void recordMetrics() {
    if (m_encoder != null) {
      SmartDashboard.putNumber("Rail.Ticks", m_encoder.getPositionTicks());
    }
    if (m_currentSensor != null) {
      SmartDashboard.putNumber("Rail.Amps", m_currentSensor.getAmps());
    }
  }

  /**
   * Internal processing on each event loop.
   */
  @Override
  public void periodic() {
    recordMetrics();
  }


  private double m_frontPositionTicks = Double.NaN;
  private double m_backPositionTicks = Double.NaN;

  private final MotorController m_motor;
  private final LimitSwitch m_frontLimitSwitch;
  private final LimitSwitch m_backLimitSwitch;
  private final Encoder m_encoder;
  private final CurrentSensor m_currentSensor;

  private final RobotConfig.ExampleRail m_railConfig;
}
