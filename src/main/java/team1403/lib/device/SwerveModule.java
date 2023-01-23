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
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig.ModuleConstants;

public class SwerveModule {
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
    private static final int CAN_TIMEOUT_MS = 250;

    private double absoluteEncoderResetIterations = 0;

    private final CougarSparkMax m_driveMotor;
    private final TalonFX m_steerMotor;

    private final CANCoder m_absoluteEncoder;
    private final double m_absoluteEncoderOffset;
    private final Encoder m_driveRelativeEncoder;
    private CougarLogger m_logger;
    private String m_name;
    
    public SwerveModule(String name, int driveMotorPort, int steerMotorNumber, int CANCoderNumber, double offset, CougarLogger logger) {
      m_logger = logger;
      m_name = name;
      m_driveMotor = CougarSparkMax.makeBrushless("DriveMotor", driveMotorPort, SparkMaxRelativeEncoder.Type.kHallSensor, logger);
      m_steerMotor = new TalonFX(steerMotorNumber);
      m_absoluteEncoder = new CANCoder(CANCoderNumber);
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
        double drivePositionConversionFactor = Math.PI * ModuleConstants.kWheelDiameterMeters * ModuleConstants.driveReduction;
        m_driveRelativeEncoder.setPositionTickConversionFactor(drivePositionConversionFactor);
        m_driveRelativeEncoder.setVelocityTickConversionFactor(drivePositionConversionFactor / 60.0);

    }

    private void configSteerMotor() {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.slot0.kP = 0.5;
        motorConfiguration.slot0.kI = 0;
        motorConfiguration.slot0.kD = 5;
        motorConfiguration.voltageCompSaturation = 12;
        motorConfiguration.supplyCurrLimit.currentLimit = 20;
        motorConfiguration.supplyCurrLimit.enable = true;

        m_steerMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS);
        m_steerMotor.enableVoltageCompensation(true);
        m_steerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS);
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
        m_driveMotor.setVoltageCompensation(12); 
        m_driveMotor.setAmpLimit(20.0);
        m_driveMotor.getCanSparkMaxApi().setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        m_driveMotor.getCanSparkMaxApi().setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        m_driveMotor.getCanSparkMaxApi().setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    }

    public double getSteerAngle() {
        double motorAngleRadians = m_steerMotor.getSelectedSensorPosition()
        * ModuleConstants.steerRelativeEncoderPositionConversionFactor;
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }
        return motorAngleRadians;
    }

    public void setControllerMode(IdleMode mode) {
      m_logger.tracef("setControllerMode %s %s", m_name, mode.toString());
      m_driveMotor.setIdleMode(mode);
    }

    public void setRampRate(double rate) {
      m_logger.tracef("setRampRate %s %f", m_name, rate);
      m_driveMotor.setOpenLoopRampRate(rate);
    }

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

    public void set(double driveVoltage, double steerAngle) {
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double difference = steerAngle - getSteerAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead
        // of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        difference = steerAngle - getSteerAngle(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can
        // be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into
            // the range [0, 2pi)
            steerAngle += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        this.m_driveMotor.setVoltage(driveVoltage);
        setReferenceAngle(steerAngle);
    }

    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = m_steerMotor.getSelectedSensorPosition()
                * ModuleConstants.steerRelativeEncoderPositionConversionFactor;

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
        // fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter
        // anymore.
        if (m_steerMotor.getSelectedSensorVelocity()
                * ModuleConstants.steerRelativeEncoderVelocityConversionFactor < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++absoluteEncoderResetIterations >= ENCODER_RESET_ITERATIONS) {
                absoluteEncoderResetIterations = 0;
                double absoluteAngle = getAbsoluteAngle();
                m_steerMotor.setSelectedSensorPosition(
                        absoluteAngle / ModuleConstants.steerRelativeEncoderPositionConversionFactor);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            absoluteEncoderResetIterations = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go
        // above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        m_steerMotor.set(TalonFXControlMode.Position,
                adjustedReferenceAngleRadians / ModuleConstants.steerRelativeEncoderPositionConversionFactor);
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

    public CANSparkMax getM_driveMotor() {
        return m_driveMotor;
    }

    public TalonFX getSteerMotorNumber() {
        return m_steerMotor;
    }

    public CANCoder getCANCoder() {
        return m_absoluteEncoder;
    }

    public Encoder getRelativeEncoder() {
        return m_driveRelativeEncoder;
    }
}