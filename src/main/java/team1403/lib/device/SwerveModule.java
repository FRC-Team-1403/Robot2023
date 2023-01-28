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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxRelativeEncoder;

import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.CougarTalonFx;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig;

/**
 * SwerveModule calling variables listed, and setting to values listed.
 */
public class SwerveModule {
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

  private SwerveDriveKinematics kDriveKinematics;


  /**
   * The method for running the swerve module.
   * @param RobotConfig
   *
   */
  public SwerveModule(String name, int driveMotorPort, int steerMotorPort, 
      int canCoderPort, double offset, CougarLogger logger, RobotConfig config) {

    m_logger = logger;
    m_name = name;

    kDriveKinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(config.swerveConfig.kTrackWidth / 2.0, config.swerveConfig.kWheelBase / 2.0),
      // Front right
      new Translation2d(config.swerveConfig.kTrackWidth / 2.0, -config.swerveConfig.kWheelBase / 2.0),
      // Back left
      new Translation2d(-config.swerveConfig.kTrackWidth / 2.0, config.swerveConfig.kWheelBase / 2.0),
      // Back right
      new Translation2d(-config.swerveConfig.kTrackWidth / 2.0, -config.swerveConfig.kWheelBase / 2.0));

    m_driveMotor = CougarSparkMax.makeBrushless("DriveMotor", driveMotorPort, 
      SparkMaxRelativeEncoder.Type.kHallSensor, logger);
    m_steerMotor = new CougarTalonFx("SteerMotor", steerMotorPort, logger);
    m_absoluteEncoder = new CANCoder(canCoderPort);
    m_driveRelativeEncoder = m_driveMotor.getEmbeddedEncoder();
    m_absoluteEncoderOffset = offset;

    configEncoders();
    configSteerMotor();
    configDriveMotor();
  }

  private void configEncoders() {
    // Config absolute encoder
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = Math.toDegrees(this.m_absoluteEncoderOffset);
    config.sensorDirection = false;
    m_absoluteEncoder.configAllSettings(config, 250);
    m_absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250);

    // Config drive relative encoder
    double drivePositionConversionFactor = 
        Math.PI * ModuleConstants.kWheelDiameterMeters * ModuleConstants.driveReduction;
    m_driveRelativeEncoder.setPositionTickConversionFactor(drivePositionConversionFactor);
    // Set velocity in terms of seconds
    m_driveRelativeEncoder.setVelocityTickConversionFactor(drivePositionConversionFactor / 60.0);

  }

  private void configSteerMotor() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.slot0.kP = RobotConfig.SwerveConfig.kP;
    motorConfiguration.slot0.kI = RobotConfig.SwerveConfig.kI;
    motorConfiguration.slot0.kD = RobotConfig.SwerveConfig.kD;
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
                getAbsoluteAngle() / ModuleConstants.steerRelativeEncoderPositionConversionFactor,
                0, CAN_TIMEOUT_MS);
    m_steerMotor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                STATUS_FRAME_GENERAL_PERIOD_MS,
                CAN_TIMEOUT_MS);
  }

  private void configDriveMotor() {
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
     * the angle for getting the steer angle.
     *
     * @return The motor angles in radians.
     *
     */
  public double getSteerAngle() {
    double motorAngleRadians = m_steerMotor.getSelectedSensorPosition()
        * ModuleConstants.steerRelativeEncoderPositionConversionFactor;
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
   * 
   */
  public void setControllerMode(IdleMode mode) {
    m_logger.tracef("setControllerMode %s %s", m_name, mode.toString());
    m_driveMotor.setIdleMode(mode);
  }

  /**
   * Sets the Ramp Rate.
   * 
   */
  public void setRampRate(double rate) {
    m_logger.tracef("setRampRate %s %f", m_name, rate);
    m_driveMotor.setOpenLoopRampRate(rate);
  }

  /**
   * method for calculating angle errors.
   *
   * @return The steer angle after accounting for error.
   *
   */
  public double angleError(double targetAngle) {
    double steerAngle = getSteerAngle();
    double difference = steerAngle - getSteerAngle();
    // Change the target angle so the difference is in the range [-pi, pi) instead
    // of [0, 2pi)
    if (difference >= Math.PI) {
      steerAngle -= 2.0 * Math.PI;
    } else if (difference < -Math.PI) {
      steerAngle += 2.0 * Math.PI;
    }
    return steerAngle - getSteerAngle();
  }

  /**
   * method for setting the drive voltage and steering angle.
   *
   * @param driveVoltage driving voltage.
   *
   * @param steerAngle steering angle.
   *
   */
  public void set(double driveVoltage, double steerAngle) {

    double trueSteerAngle = steerAngle;
    double trueDriveVoltage = driveVoltage;

    trueSteerAngle %= (2.0 * Math.PI);
    if (trueSteerAngle < 0.0) {
      trueSteerAngle += 2.0 * Math.PI;
    }

    double difference = trueSteerAngle - getSteerAngle();
    // Change the target angle so the difference is in the range [-pi, pi) instead
    // of [0, 2pi)
    if (difference >= Math.PI) {
      trueSteerAngle -= 2.0 * Math.PI;
    } else if (difference < -Math.PI) {
      trueSteerAngle += 2.0 * Math.PI;
    }
    difference = trueSteerAngle - getSteerAngle(); // Recalculate difference

    // If the difference is greater than 90 deg or less than -90 deg the drive can
    // be inverted so the total
    // movement of the module is less than 90 deg
    if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
      // Only need to add 180 deg here because the target angle will be put back into
      // the range [0, 2pi)
      trueSteerAngle += Math.PI;
      trueDriveVoltage *= -1.0;
    }

    // Put the target angle back into the range [0, 2pi)
    trueSteerAngle %= (2.0 * Math.PI);
    if (trueSteerAngle < 0.0) {
      trueSteerAngle += 2.0 * Math.PI;
    }

    this.m_driveMotor.setVoltage(trueDriveVoltage);
    setReferenceAngle(trueSteerAngle);
  }

  /**
   * it sets the reference angle.
   *
   * @param referenceAngleRadians the parameter for radian angle.
   *
   */
  public void setReferenceAngle(double referenceAngleRadians) {
    double currentAngleRadians = m_steerMotor.getSelectedSensorPosition()
                * ModuleConstants.steerRelativeEncoderPositionConversionFactor;

    // Reset the NEO's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
    // fully set up, and we don't
    // end up getting a good reading. If we reset periodically this won't matter
    // anymore.
    if (m_steerMotor.getSelectedSensorVelocity()
         * ModuleConstants.steerRelativeEncoderVelocityConversionFactor 
            < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
      if (++m_absoluteEncoderResetIterations >= ENCODER_RESET_ITERATIONS) {
        m_absoluteEncoderResetIterations = 0;
        double absoluteAngle = getAbsoluteAngle();
        m_steerMotor.setSelectedSensorPosition(absoluteAngle 
                    / ModuleConstants.steerRelativeEncoderPositionConversionFactor);
        currentAngleRadians = absoluteAngle;
      }
    } else {
      m_absoluteEncoderResetIterations = 0;
    }

    double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
    if (currentAngleRadiansMod < 0.0) {
      currentAngleRadiansMod += 2.0 * Math.PI;
    }

    // The reference angle has the range [0, 2pi) but the Falcon's encoder can go
    // above that
    double adjustedReferenceAngleRadians = 
        referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
    if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
      adjustedReferenceAngleRadians -= 2.0 * Math.PI;
    } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
      adjustedReferenceAngleRadians += 2.0 * Math.PI;
    }

    m_steerMotor.set(TalonFXControlMode.Position, adjustedReferenceAngleRadians 
            / ModuleConstants.steerRelativeEncoderPositionConversionFactor);
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
   * Returns the SwerveModulePosition
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(m_driveRelativeEncoder.getPositionTicks(), new Rotation2d(getSteerAngle()));
  }
}