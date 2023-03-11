package team1403.robot.chargedup.swerve;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.wpi.NavxAhrs;
import team1403.lib.util.CougarLogger;
import team1403.lib.util.SwerveDriveOdometry;
import team1403.lib.util.SwerveDrivePoseEstimator;
import team1403.robot.chargedup.RobotConfig.CanBus;
import team1403.robot.chargedup.RobotConfig.SwerveConfig;

/**
 * The drivetrain of the robot. Consists of for swerve modules and the
 * gyroscope.
 */
public class SwerveSubsystem extends CougarSubsystem {
  private final NavxAhrs m_navx2;
  private final SwerveModule[] m_modules;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private SwerveModuleState[] m_states = new SwerveModuleState[4];
  private final SwerveDrivePoseEstimator m_odometer;

  private final PIDController m_driftCorrectionPid = new PIDController(0.35, 0, 0);
  private double m_desiredHeading = 0;
  private double m_speedLimiter = 0.6;

  private Translation2d m_offset;

  private double m_calc = 0;

  /**
   * Creates a new {@link SwerveSubsystem}.
   * Instantiates the 4 {@link SwerveModule}s,
   * the {@link SwerveDriveOdometry}, and the {@link NavxAhrs}.
   * Also sets drivetrain ramp rate,
   * and idle mode to default values.
   *
   * @param parameters the {@link CougarLibInjectedParameters}
   *                   used to construct this subsystem
   */
  public SwerveSubsystem(CougarLibInjectedParameters parameters) {
    super("Swerve Subsystem", parameters);
    CougarLogger logger = getLogger();
    m_navx2 = new NavxAhrs("Gyroscope");
    m_modules = new SwerveModule[] {
        new SwerveModule("Front Left Module",
            CanBus.frontLeftDriveId, CanBus.frontLeftSteerId,
            CanBus.frontLeftEncoderId, SwerveConfig.frontLeftEncoderOffset, logger),
        new SwerveModule("Front Right Module",
            CanBus.frontRightDriveId, CanBus.frontRightSteerId,
            CanBus.frontRightEncoderId, SwerveConfig.frontRightEncoderOffset, logger),
        new SwerveModule("Back Left Module",
            CanBus.backLeftDriveId, CanBus.backLeftSteerId,
            CanBus.backLeftEncoderId, SwerveConfig.backLeftEncoderOffset, logger),
        new SwerveModule("Back Right Module",
            CanBus.backRightDriveId, CanBus.backRightSteerId,
            CanBus.backRightEncoderId, SwerveConfig.backRightEncoderOffset, logger),
    };

    m_odometer = new SwerveDrivePoseEstimator(SwerveConfig.kDriveKinematics, getGyroscopeRotation(),
        getModulePositions(), new Pose2d(1.7272, 5.9436, new Rotation2d(0)));

    addDevice(m_navx2.getName(), m_navx2);
    new Thread(() -> {
      while (m_navx2.isCalibrating()) {
        try {
          Thread.sleep(10);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
      zeroGyroscope();
    }).start();

    m_desiredHeading = getGyroscopeRotation().getDegrees();

    setRobotRampRate(0.0);
    setRobotIdleMode(IdleMode.kCoast);

    m_offset = new Translation2d();
    
  }

  /**
   * Increases the speed limiter by amt. The speed limiter will not exceed 1.
   *
   * @param amt the amount to increase the speed by
   */
  public void increaseSpeed(double amt) {
    tracef("increasedSpeed %f", amt);
    m_speedLimiter = Math.min(1, m_speedLimiter + amt);
  }

  /**
   * Decreases the speed limiter by amt. The speed limiter will not go below 0.
   *
   * @param amt the amount to decrease the speed by
   */
  public void decreaseSpeed(double amt) {
    tracef("decreasedSpeed %f", amt);
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
    tracef("setRobotRampRate %f", rate);
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
    tracef("setRobotIdleMode %s", mode.name());
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
    // tracef("zeroGyroscope %f", getGyroscopeRotation());
    m_navx2.reset();
    m_desiredHeading = 0;
  }

  /**
   * Return the position of the drivetrain.
   *
   * @return the position of the drivetrain in Pose2d
   */
  public Pose2d getPose() {
    return m_odometer.getEstimatedPosition();
  }

  /**
   * Set the position of thte odometry.
   *
   * @param pose the new position of the odometry.
   */
  public void setPose(Pose2d pose) {
    m_odometer.setPose(pose);
  }

  public SwerveDrivePoseEstimator getOdometer() {
    return m_odometer;
  }

  public void updateOdometerWithVision(Pose2d pose) {
    if (pose.getTranslation().getDistance(getPose().getTranslation()) < 1) {
      m_odometer.addVisionMeasurement(pose, Timer.getFPGATimestamp());
    }
  }

  /**
   * Reset the position of the drivetrain odometry.
   */
  public void resetOdometry() {
    tracef("resetOdometry");
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
   * Gets the roll of the gyro (Y axis of gyro rotation).
   * 
   * @return a double representing the roll of robot in degrees
   */
  public double getGyroRoll() {
    return m_navx2.getRoll();
  }

  /**
   * Gets the pitch of the gyro (X axis of gyro rotation).
   * 
   * @return a double representing the pitch of robot in degrees
   */
  public double getGyroPitch() {
    return m_navx2.getPitch();
  }

  /**
   * Moves the drivetrain at the given chassis speeds.
   *
   * @param chassisSpeeds the speed to move at
   * @param offset the swerve module to pivot around
   */
  public void drive(ChassisSpeeds chassisSpeeds, Translation2d offset) {
    m_chassisSpeeds = chassisSpeeds;
    m_offset = offset;
    SmartDashboard.putString("Chassis Speeds", m_chassisSpeeds.toString());
  }

  /**
   * Stops the drivetrain.
   */
  public void stop() {
    tracef("stop");
    m_chassisSpeeds = new ChassisSpeeds();
  }

  public Pose2d getOdometryValue() {
    return m_odometer.getEstimatedPosition();
  }

  /**
   * Sets the module speed and heading for all 4 modules.
   *
   * @param states an array of states for each module.
   */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, SwerveConfig.kMaxSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].set((states[i].speedMetersPerSecond
          / SwerveConfig.kMaxSpeed) * m_speedLimiter,
          states[i].angle.getRadians());
    }
  }

  /**
   * Adds rotational velocity to the chassis speed to compensate for
   * unwanted changes in gyroscope heading.
   * 
   * @param chassisSpeeds the given chassisspeeds
   * @return the corrected chassisspeeds
   */
  private ChassisSpeeds translationalDriftCorrection(ChassisSpeeds chassisSpeeds) {

    double translationalVelocity = Math.abs(m_modules[0].getDriveVelocity());
    if (Math.abs(m_navx2.getAngularVelocity()) > 0.1) {
      m_desiredHeading = getGyroscopeRotation().getDegrees();
    } else if (translationalVelocity > 1) {
      m_calc = m_driftCorrectionPid.calculate(getGyroscopeRotation().getDegrees(),
          m_desiredHeading);
      if (Math.abs(m_calc) >= 0.55) {
        m_chassisSpeeds.omegaRadiansPerSecond += m_calc;
      }
      tracef("driftCorrection %f, corrected omegaRadiansPerSecond %f",
          m_calc, m_chassisSpeeds.omegaRadiansPerSecond);
    }
    return chassisSpeeds;
  }

  /**
   * Accounts for the drift caused by the first order kinematics
   * while doing both translational and rotational movement.
   * 
   * <p>
   * Looks forward one control loop to figure out where the robot
   * should be given the chassisspeed and backs out a twist command from that.
   * 
   * @param chassisSpeeds the given chassisspeeds
   * @return the corrected chassisspeeds
   */
  private ChassisSpeeds rotationalDriftCorrection(ChassisSpeeds chassisSpeeds) {
    // Assuming the control loop runs in 20ms
    final double deltaTime = 0.02;

    // The position of the bot one control loop in the future given the chassisspeed
    Pose2d robotPoseVel = new Pose2d(chassisSpeeds.vxMetersPerSecond * deltaTime,
        chassisSpeeds.vyMetersPerSecond * deltaTime,
        new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * deltaTime));

    Twist2d twistVel = new Pose2d(0, 0, new Rotation2d()).log(robotPoseVel);
    return new ChassisSpeeds(
        twistVel.dx / deltaTime, twistVel.dy / deltaTime,
        twistVel.dtheta / deltaTime);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Reading", getGyroscopeRotation().getDegrees());
    m_odometer.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation(), getModulePositions());
    SmartDashboard.putString("Odometry", m_odometer.getEstimatedPosition().toString());
    SmartDashboard.putNumber("Speed", m_speedLimiter);
        
    m_chassisSpeeds = translationalDriftCorrection(m_chassisSpeeds);
    m_chassisSpeeds = rotationalDriftCorrection(m_chassisSpeeds);


    SmartDashboard.putNumber("Front Left Absolute Encoder",  m_modules[0].getAbsoluteAngle());
    SmartDashboard.putNumber("Front Right Absolute Encoder",  m_modules[1].getAbsoluteAngle());
    SmartDashboard.putNumber("Back Left Absolute Encoder",  m_modules[2].getAbsoluteAngle());
    SmartDashboard.putNumber("Back Right Absolute Encoder",  m_modules[3].getAbsoluteAngle());

    m_states = SwerveConfig.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds, m_offset);

    setModuleStates(m_states);
  }
}
