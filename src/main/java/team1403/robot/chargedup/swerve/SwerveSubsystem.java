package team1403.robot.chargedup.swerve;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import team1403.lib.device.GyroscopeDevice;
import team1403.lib.device.wpi.NavxAhrs;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig.CanBus;
import team1403.robot.chargedup.RobotConfig.SwerveConfig;

/**
 * The drivetrain of the robot. Consists of for swerve modules and the
 * gyroscope.
 */
public class SwerveSubsystem extends SubsystemBase {
  private final GyroscopeDevice m_navx2;
  private final SwerveModule[] m_modules;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private final SwerveDriveOdometry m_odometer;

  private PIDController m_driftCorrectionPid = new PIDController(0.33, 0, 0);
  private double m_desiredHeading = 0;
  private double m_speedLimiter = 0.6;

  private final CougarLogger m_logger;

  /**
   * Creates a new {@link SwerveSubsystem}. Instantiates the 4 {@link SwerveModule}s, 
   * the {@link SwerveDriveOdometry}, and the {@link NavxAhrs}. 
   * Also sets drivetrain ramp rate, and idle mode to default values.
   *
   * @param logger the {@link CougarLogger} used in this subsystem
   */
  public SwerveSubsystem(CougarLogger logger) {
    this.m_logger = logger;
    m_modules = new SwerveModule[] {
        new SwerveModule("Back Right Module",
            CanBus.backRightDriveId, CanBus.backRightSteerId,
            CanBus.backRightEncoderId, SwerveConfig.backRightEncoderOffset, logger),
        new SwerveModule("Back Left Module",
            CanBus.backLeftDriveId, CanBus.backLeftSteerId,
            CanBus.backLeftEncoderId, SwerveConfig.backLeftEncoderOffset, logger),
        new SwerveModule("Front Left Module",
            CanBus.frontLeftDriveId, CanBus.frontLeftSteerId,
            CanBus.frontLeftEncoderId, SwerveConfig.frontLeftEncoderOffset, logger),
        new SwerveModule("Front Right Module",
            CanBus.frontRightDriveId, CanBus.frontRightSteerId,
            CanBus.frontRightEncoderId, SwerveConfig.frontRightEncoderOffset, logger)
    };

    m_odometer = new SwerveDriveOdometry(
        SwerveConfig.kDriveKinematics,
        new Rotation2d(), getModulePositions(), new Pose2d());

    m_navx2 = new NavxAhrs("Gyroscope");

    m_desiredHeading = getGyroscopeRotation().getDegrees();

    setRobotRampRate(0.0);
    setRobotIdleMode(IdleMode.kBrake);
  }

  /**
   * Increases the speed limiter by amt. The speed limiter will not exceed 1.
   *
   * @param amt the amount to increase the speed by
   */
  public void increaseSpeed(double amt) {
    m_speedLimiter = Math.min(1, m_speedLimiter + amt);
  }

  /**
   * Decreases the speed limiter by amt. The speed limiter will not go below 0.
   *
   * @param amt the amount to decrease the speed by
   */
  public void decreaseSpeed(double amt) {
    m_speedLimiter = Math.max(0, m_speedLimiter - amt);
  }

  /**
   * Gets the 4 swerve module positions.
   *
   * @return an array of swerve module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_modules[0].getPosition(),
        m_modules[1].getPosition(),
        m_modules[2].getPosition(),
        m_modules[3].getPosition()
    };
  }

  /**
   * Sets the ramp rate of the drive motors.
   *
   * @param rate the ramp rate
   */
  public void setRobotRampRate(double rate) {
    for (SwerveModule module : m_modules) {
      module.setRampRate(rate);
    }
  }

  /**
   * Sets the idle mode for the drivetrain.
   *
   * @param mode the IdleMode of the robot
   */
  public void setRobotIdleMode(IdleMode mode) {
    for (SwerveModule module : m_modules) {
      module.setControllerMode(mode);
    }
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_navx2.reset();
    m_desiredHeading = 0;
    resetOdometry();
  }

  /**
   * Return the position of the drivetrain.
   *
   * @return the position of the drivetrain in Pose2d
   */
  public Pose2d getPose() {
    return m_odometer.getPoseMeters();
  }

  /**
   * Reset the position of the drivetrain odometry.
   */
  public void resetOdometry() {
    m_odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), getPose());
  }

  /**
   * Gets the heading of the gyroscope.
   *
   * @return a Rotation2d object that contains the gyroscope's heading
   */
  public Rotation2d getGyroscopeRotation() {
    return m_navx2.getRotation2d();
  }

  /**
   * Moves the drivetrain at the given chassis speeds.
   *
   * @param chassisSpeeds the speed to move at
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  /**
   * Sets the module speed and heading for all 4 modules.
   *
   * @param states an array of states for each module.
   */
  public void setModuleStates(SwerveModuleState[] states) {
    // Prevent wheels from going back to 0 degrees as the default state.
    if (states[0].speedMetersPerSecond < 0.001) {
      for (int i = 0; i < m_modules.length; i++) {
        m_modules[i].set(0, m_modules[i].getSteerAngle());
      }
      return;
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConfig.kMaxSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].set((states[i].speedMetersPerSecond
          / SwerveConfig.kMaxSpeed) * m_speedLimiter, states[i].angle.getRadians());
    }
  }

  /**
   * Adds rotational velocity to the chassis speed to compensate for 
   * unwanted changes in gyroscope heading.
   */
  private void driftCorrection() {
    double translationalVelocity = Math.abs(m_modules[0].getDriveVelocity());
    if (Math.abs(m_navx2.getAngularVelocity()) > 0.2) {
      m_desiredHeading = getGyroscopeRotation().getDegrees();
    } else if (translationalVelocity > 0) {
      double calc = m_driftCorrectionPid.calculate(getGyroscopeRotation().getDegrees(),
          m_desiredHeading);
      if (Math.abs(calc) >= 0.55) {
        m_chassisSpeeds.omegaRadiansPerSecond += calc;
      }
    }
  }

  /**
   * Stops the drivetrain.
   */
  public void stop() {
    m_chassisSpeeds = new ChassisSpeeds();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Reading", getGyroscopeRotation().getDegrees());
    m_odometer.update(getGyroscopeRotation(), getModulePositions());
    SmartDashboard.putString("Odometry", m_odometer.getPoseMeters().toString());

    driftCorrection();

    SwerveModuleState[] states = SwerveConfig.kDriveKinematics
        .toSwerveModuleStates(m_chassisSpeeds);

    setModuleStates(states);
  }
}
