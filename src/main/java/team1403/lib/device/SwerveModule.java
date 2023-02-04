package team1403.lib.device;

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

import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.CougarTalonFx;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig;
import team1403.robot.chargedup.RobotConfig.SwerveConfig;

/**
 * SwerveModule calling variables listed, and setting to values listed.
 */
public class SwerveModule implements Device {
  private static final int ENCODER_RESET_ITERATIONS = 500;
  private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
  private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private static final int CAN_TIMEOUT_MS = 250;

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
    m_absoluteEncoder.configAllSettings(config, CAN_TIMEOUT_MS);
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

    m_steerMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS);
    m_steerMotor.enableVoltageCompensation(true);
    m_steerMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS);
    m_steerMotor.setSensorPhase(true);
    m_steerMotor.setInverted(TalonFXInvertType.CounterClockwise);
    m_steerMotor.setNeutralMode(NeutralMode.Brake);
    m_steerMotor.setSelectedSensorPosition(
        getAbsoluteAngle() / SwerveConfig.kSteerRelativeEncoderPositionConversionFactor,
        0, CAN_TIMEOUT_MS);
    m_steerMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General,
        STATUS_FRAME_GENERAL_PERIOD_MS,
        CAN_TIMEOUT_MS);
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
   * The angle for getting the steer angle.
   *
   * @return The motor angles in radians.
   */
  public double getSteerAngle() {
    double motorAngleRadians = m_steerMotor.getSelectedSensorPosition()
        * SwerveConfig.kSteerRelativeEncoderPositionConversionFactor;
    motorAngleRadians %= 2.0 * Math.PI;
    if (motorAngleRadians < 0) {
      motorAngleRadians += 2.0 * Math.PI;
    }
    return motorAngleRadians;
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
   * Sets the Ramp Rate.
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
  public double normalizeAngle(double angle) {
    double normalizedAngle = angle;

    normalizedAngle %= (2.0 * Math.PI);
    if (normalizedAngle < 0.0) {
      normalizedAngle += 2.0 * Math.PI;
    }
    return normalizedAngle;
  }

  /**
   * method for calculating angle errors.
   *
   * @param targetAngle the angle to be moved to
   * @return The steer angle after accounting for error.
   */
  public double normalizeAngleError(double targetAngle) {

    double normalizedAngleError = targetAngle;

    // Angle is inbetween 0 to 2pi
    normalizedAngleError = normalizeAngle(normalizedAngleError);

    double difference = normalizedAngleError - getSteerAngle();
    // Change the target angle so the difference is in the range [-pi, pi) instead
    // of [0, 2pi)
    if (difference >= Math.PI) {
      normalizedAngleError -= 2.0 * Math.PI;
    } else if (difference < -Math.PI) {
      normalizedAngleError += 2.0 * Math.PI;
    }
    return normalizedAngleError - getSteerAngle();
  }

  /**
   * Converts the steer angle to the next angle the swerve module should turn to.
   *
   * @param steerAngle the current steer angle.
   */
  public double convertSteerAngle(double steerAngle) {

    double newSteerAngle = steerAngle;

    double difference = normalizeAngleError(newSteerAngle);

    // If the difference is greater than 90 deg or less than -90 deg the drive can
    // be inverted so the total
    // movement of the module is less than 90 deg
    if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
      // Only need to add 180 deg here because the target angle will be put back into
      // the range [0, 2pi)
      newSteerAngle += Math.PI;
    }

    // Put the target angle back into the range [0, 2pi)
    newSteerAngle = normalizeAngle(newSteerAngle);

    // Angle to be changed is now in radians
    double referenceAngleRadians = newSteerAngle;
    double currentAngleRadians = m_steerMotor.getSelectedSensorPosition();

    // Reset the NEO's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
    // fully set up, and we don't
    // end up getting a good reading. If we reset periodically this won't matter
    // anymore.
    if (m_steerMotor.getSelectedSensorVelocity()
        * SwerveConfig.kSteerRelativeEncoderVelocityConversionFactor 
            < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
      if (++m_absoluteEncoderResetIterations >= ENCODER_RESET_ITERATIONS) {
        m_absoluteEncoderResetIterations = 0;
        double absoluteAngle = getAbsoluteAngle();
        m_steerMotor.setSelectedSensorPosition(getAbsoluteAngle()
            / SwerveConfig.kSteerRelativeEncoderPositionConversionFactor);
        currentAngleRadians = absoluteAngle;
      }
    } else {
      m_absoluteEncoderResetIterations = 0;
    }

    double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
    if (currentAngleRadiansMod < 0.0) {
      currentAngleRadiansMod += 2.0 * Math.PI;
    }

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
  public double convertDriveMetersPerSecond(double driveMetersPerSecond, double steerAngle) {

    double convertedDriveMetersPerSecond = driveMetersPerSecond;

    double difference = normalizeAngleError(steerAngle);

    // If the difference is greater than 90 deg or less than -90 deg the drive can
    // be inverted so the total
    // movement of the module is less than 90 deg
    if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
      // Only need to add 180 deg here because the target angle will be put back into
      // the range [0, 2pi)
      convertedDriveMetersPerSecond *= -1.0;
    }

    return convertedDriveMetersPerSecond;
  }

  /**
   * Method for setting the drive voltage and steering angle.
   *
   * @param driveMetersPerSecond driving meters per second.
   *
   * @param steerAngle           steering angle.
   *
   */
  public void set(double driveMetersPerSecond, double steerAngle) {

    // Set driveMotor according to voltage
    this.m_driveMotor.set(convertDriveMetersPerSecond(driveMetersPerSecond, steerAngle));

    // Set steerMotor according to position of encoder
    this.m_steerMotor.set(TalonFXControlMode.Position, convertSteerAngle(steerAngle));
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
   * intializes the drive motor.
   *
   * @return drive motor value?.
   * 
   */
  public CANSparkMax getdriveMotor() {
    return m_driveMotor;
  }

  /**
   * Returns the steer motor object.
   *
   * @return the steermotor object.
   * 
   */
  public TalonFX getSteerMotor() {
    return m_steerMotor;
  }

  /**
   * Gets CANCoder value.
   *
   * @return the CANCoder value.
   * 
   */
  public CANCoder getAbsoluteEncoder() {
    return m_absoluteEncoder;
  }

  /**
   * Gets the value from the relative encoder.
   *
   * @return relative encoder value.
   *
   */
  public Encoder getRelativeEncoder() {
    return m_driveRelativeEncoder;
  }

  /**
   * Returns the SwerveModulePosition.
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(m_driveRelativeEncoder.getPositionTicks(), 
          new Rotation2d(getSteerAngle()));
  }
}