package team1403.robot.chargedup.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.device.Device;
import team1403.lib.device.Encoder;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.CougarTalonFx;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig;
import team1403.robot.chargedup.RobotConfig.SwerveConfig;

/**
 * Represents a swerve module. Consists of a drive motor, steer motor, 
 * and their respective relative encoders. Also consists of a absolute encoder to track steer angle.
 */
public class SwerveModule implements Device {
  private double m_absoluteEncoderResetIterations = 0;

  private final CougarSparkMax m_driveMotor;
  private final CougarTalonFx m_steerMotor;

  private final CANCoder m_absoluteEncoder;
  private final double m_absoluteEncoderOffset;
  private final Encoder m_driveRelativeEncoder;
  private final CougarLogger m_logger;
  private final String m_name;

  /**
   * Swerve Module represents a singular swerve module for a
   * swerve drive train.
   * 
   * <p>Each swerve module consists of a drive motor,
   * changing the velocity of the wheel, and a steer motor, changing
   * the angle of the actual wheel inside of the module.
   * 
   * <p>The swerve module also features
   * an absolute encoder to ensure the angle of
   * the module is always known, regardless if the bot is turned off
   * or not.
   *
   */
  public SwerveModule(String name, int driveMotorPort, int steerMotorPort,
      int canCoderPort, double offset, CougarLogger logger) {

    m_logger = logger;
    m_name = name;

    m_driveMotor = CougarSparkMax.makeBrushless("DriveMotor", driveMotorPort,
        SparkMaxRelativeEncoder.Type.kHallSensor, logger);
    m_steerMotor = new CougarTalonFx("SteerMotor", steerMotorPort, logger);
    m_absoluteEncoder = new CANCoder(canCoderPort);
    m_driveRelativeEncoder = m_driveMotor.getEmbeddedEncoder();
    m_absoluteEncoderOffset = offset;

    initEncoders();
    initSteerMotor();
    initDriveMotor();
  }

  @Override
  public String getName() {
    return m_name;
  }

  private void initEncoders() {
    // Config absolute encoder
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = Math.toDegrees(this.m_absoluteEncoderOffset);
    config.sensorDirection = false;
    m_absoluteEncoder.configAllSettings(config, SwerveConfig.kCanTimeoutMs);
    m_absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250);

    // Config drive relative encoder
    double drivePositionConversionFactor = Math.PI * SwerveConfig.kWheelDiameterMeters 
          * SwerveConfig.kDriveReduction;
    m_driveRelativeEncoder.setPositionTickConversionFactor(drivePositionConversionFactor);
    // Set velocity in terms of seconds
    m_driveRelativeEncoder.setVelocityTickConversionFactor(drivePositionConversionFactor / 60.0);

  }

  private void initSteerMotor() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.slot0.kP = RobotConfig.SwerveConfig.kPTurning;
    motorConfiguration.slot0.kI = RobotConfig.SwerveConfig.kITurning;
    motorConfiguration.slot0.kD = RobotConfig.SwerveConfig.kDTurning;
    motorConfiguration.voltageCompSaturation = RobotConfig.SwerveConfig.kVoltageSaturation;
    motorConfiguration.supplyCurrLimit.currentLimit = RobotConfig.SwerveConfig.kCurrentLimit;
    motorConfiguration.supplyCurrLimit.enable = true;

    m_steerMotor.configAllSettings(motorConfiguration, SwerveConfig.kCanTimeoutMs);
    m_steerMotor.enableVoltageCompensation(true);
    m_steerMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, 0, SwerveConfig.kCanTimeoutMs);
    m_steerMotor.setSensorPhase(true);
    m_steerMotor.setInverted(TalonFXInvertType.CounterClockwise);
    m_steerMotor.setNeutralMode(NeutralMode.Brake);
    m_steerMotor.setSelectedSensorPosition(
        getAbsoluteAngle() / SwerveConfig.kSteerRelativeEncoderPositionConversionFactor,
        0, SwerveConfig.kCanTimeoutMs);
    m_steerMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General,
        SwerveConfig.kStatusFrameGeneralPeriodMs,
        SwerveConfig.kCanTimeoutMs);
  }

  private void initDriveMotor() {
    m_driveMotor.setInverted(true);
    m_driveMotor.setVoltageCompensation(RobotConfig.SwerveConfig.kVoltageSaturation);
    m_driveMotor.setAmpLimit(RobotConfig.SwerveConfig.kCurrentLimit);
    m_driveMotor.getCanSparkMaxApi().setPeriodicFramePeriod(
        CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_driveMotor.getCanSparkMaxApi().setPeriodicFramePeriod(
        CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_driveMotor.getCanSparkMaxApi().setPeriodicFramePeriod(
        CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
  }

  /**
   * The method for getting the steer angle.
   *
   * @return The motor angles in radians.
   */
  public double getSteerAngle() {
    double motorAngleRadians = m_steerMotor.getSelectedSensorPosition()
        * SwerveConfig.kSteerRelativeEncoderPositionConversionFactor;
    return normalizeAngle(motorAngleRadians);
  }

  /**
   * Sets the contoller mode.
   *
   * @param mode its the mode for the controller.
   */
  public void setControllerMode(IdleMode mode) {
    m_logger.tracef("setControllerMode %s %s", m_name, mode.toString());
    m_driveMotor.setIdleMode(mode);
  }

  /**
   * Sets the ramp rate.
   *
   * @param rate speed in seconds motor will take to ramp to speed
   */
  public void setRampRate(double rate) {
    m_logger.tracef("setRampRate %s %f", m_name, rate);
    m_driveMotor.setOpenLoopRampRate(rate);
  }

  /**
   * Normalizes angle value to be inbetween values 0 to 2pi.
   *
   * @param angle angle to be normalized
   * @return angle value between 0 to 2pi
   */
  private double normalizeAngle(double angle) {
    angle %= (2.0 * Math.PI);
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }
    return angle;
  }

  /**
   * Returns difference (targetAngle - getSteerAngle()) normalized in range -pi .. pi
   *
   * @param targetAngle the angle to be moved to
   * @return The steer angle after accounting for error.
   */
  private double normalizeAngleError(double targetAngle) {
    // Angle is inbetween 0 to 2pi

    double difference = targetAngle - getSteerAngle();
    // Change the target angle so the difference is in the range [-pi, pi) instead
    // of [0, 2pi)
    if (difference >= Math.PI) {
      targetAngle -= 2.0 * Math.PI;
    } else if (difference < -Math.PI) {
      targetAngle += 2.0 * Math.PI;
    } 
    return targetAngle - getSteerAngle();
  }

  /**
   * Converts the steer angle to the next angle the swerve module should turn to.
   *
   * @param steerAngle the current steer angle.
   */
  private double convertSteerAngle(double steerAngle) {
    steerAngle = normalizeAngle(steerAngle);
    double difference = normalizeAngleError(steerAngle);        

    // If the difference is greater than 90 deg or less than -90 deg the drive can
    // be inverted so the total
    // movement of the module is less than 90 deg
    if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
      // Only need to add 180 deg here because the target angle will be put back into
      // the range [0, 2pi)
      steerAngle += Math.PI;
    }

    // Put the target angle back into the range [0, 2pi)
    steerAngle = normalizeAngle(steerAngle);

    // Angle to be changed is now in radians
    double referenceAngleRadians = steerAngle;
    double currentAngleRadians = m_steerMotor.getSelectedSensorPosition() * SwerveConfig.kSteerRelativeEncoderPositionConversionFactor;

    // Reset the NEO's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
    // fully set up, and we don't
    // end up getting a good reading. If we reset periodically this won't matter
    // anymore.
    if (m_steerMotor.getSelectedSensorVelocity()
        * SwerveConfig.kSteerRelativeEncoderVelocityConversionFactor 
            < SwerveConfig.kEncoderResetMaxAngularVelocity) {
      if (++m_absoluteEncoderResetIterations >= SwerveConfig.kEncoderResetIterations) {
        m_logger.tracef("Resetting steer relative encoder. Reset iteration %f", 
            m_absoluteEncoderResetIterations);
        m_absoluteEncoderResetIterations = 0;
        double absoluteAngle = getAbsoluteAngle();
        m_steerMotor.setSelectedSensorPosition(getAbsoluteAngle()
            / SwerveConfig.kSteerRelativeEncoderPositionConversionFactor);
        currentAngleRadians = absoluteAngle;
      }
    } else {
      m_absoluteEncoderResetIterations = 0;
    }

    double currentAngleRadiansMod = normalizeAngle(currentAngleRadians);

    // The reference angle has the range [0, 2pi)
    // but the Falcon's encoder can go above that
    double adjustedReferenceAngleRadians = referenceAngleRadians 
        + currentAngleRadians - currentAngleRadiansMod;
    if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
      adjustedReferenceAngleRadians -= 2.0 * Math.PI;
    } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
      adjustedReferenceAngleRadians += 2.0 * Math.PI;
    }

    // The position that the motor should turn to
    // when taking into account the ticks of the motor
    return adjustedReferenceAngleRadians 
      / SwerveConfig.kSteerRelativeEncoderPositionConversionFactor;
  }

  /**
   * Converts the drive votlage to be inverted or not.
   *
   * @param steerAngle          the current steer angle.
   * @param driveMetersPerSecond the current drive voltage
   */
  private double convertDriveMetersPerSecond(double driveMetersPerSecond, double steerAngle) {

    double difference = normalizeAngleError(steerAngle);

    // If the difference is greater than 90 deg or less than -90 deg the drive can
    // be inverted so the total
    // movement of the module is less than 90 deg
    if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
      // Only need to add 180 deg here because the target angle will be put back into
      // the range [0, 2pi)
      driveMetersPerSecond *= -1.0;
    }

    return driveMetersPerSecond;
  }

  /**
   * Method for setting the drive voltage and steering angle.
   *
   * @param driveMetersPerSecond driving meters per second.
   * @param steerAngle           steering angle.
   *
   */
  public void set(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, new Rotation2d(getSteerAngle()));
    // Set driveMotor according to percentage output
    this.m_driveMotor.set(state.speedMetersPerSecond);
    // Set steerMotor according to position of encoder
    this.m_steerMotor.set(TalonFXControlMode.Position, state.angle.getRadians());
  }

  /**
   * Gets the current angle reading of the encoder in radians.
   *
   * @return The current angle in radians. Range: [0, 2pi)
   */
  public double getAbsoluteAngle() {
    double angle = Math.toRadians(m_absoluteEncoder.getAbsolutePosition());
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }
    return angle;
  }

  /**
   * Returns the drive motor object associated with the module.
   *
   * @return the drive motor object.
   * 
   */
  public CANSparkMax getDriveMotor() {
    return m_driveMotor;
  }

  /**
   * Returns the steer motor object associated with the module.
   *
   * @return the steer motor object.
   * 
   */
  public TalonFX getSteerMotor() {
    return m_steerMotor;
  }

  /**
   * Returns the CANCoder object associated with the module.
   *
   * @return the CANCoder object.
   * 
   */
  public CANCoder getAbsoluteEncoder() {
    return m_absoluteEncoder;
  }

  /**
   * Returns the relative encoder for the drive motor.
   *
   * @return relative encoder value from drive motor.
   *
   */
  public Encoder getDriveRelativeEncoder() {
    return m_driveRelativeEncoder;
  }

  /**
   * Returns the SwerveModulePosition of this particular module.
   *
   * @return the SwerveModulePosition, which represents the distance 
   *         travelled and the angle of the module.
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(m_driveRelativeEncoder.getPositionTicks(), 
          new Rotation2d(getSteerAngle()));
  }
  
  /**
   * Returns the current velocity of the drive motor.
   *
   * @return the current velocity of the drive motor
   */
  public double getDriveVelocity() {
    return m_driveRelativeEncoder.getVelocityTicks();
  }

  /**
   * Returns the current state of the swerve module as defined by 
   * the relative encoders of the drive and steer motors.
   *
   * @return the current state of the swerve module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
  }

  /**
   * Returns the current position of the swerve module as defined by 
   * the relative encoders of the drive and steer motors.
   *
   * @return the current position of the swerve module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getEncoder().getPosition(), 
        new Rotation2d(getSteerAngle()));
  }
}