package team1403.robot.chargedup.swerve;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The drivetrain of the robot. Consists of for swerve modules and the gyroscope.
 */
public class SwerveSubsystem extends SubsystemBase {

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * 
   * <p>This can be reduced to cap the robot's maximum speed. 
   * Typically, this is useful during initial testing of the robot.
  */
  public static double MAX_VOLTAGE = 5.0;

	private final GyrosAHScope m_navx2 = Gyroscope.getInstance();

	private final SwerveDriveOdometry m_odometer = new SwerveDriveOdometry(
			Constants.m_kinematics,
			new Rotation2d());

	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();

	private PIDController driftCorrectionPID = new PIDController(0.33
	, 0, 0); /* I and D just make it worse */
	private double XY = 0;
	private double pXY = 0;
	private double desiredHeading = 0;

	private static SwerveSubsystem instance = null;
	private SwerveSubsystem() {

		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
		m_backRightModule = new SwerveModule(
			tab.getLayout("Back Right Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(6, 0),
			BACK_RIGHT_MODULE_DRIVE_MOTOR,
			BACK_RIGHT_MODULE_STEER_MOTOR,
			BACK_RIGHT_MODULE_STEER_ENCODER,
			BACK_RIGHT_MODULE_STEER_OFFSET, 0.5,
			0, 
			5);
		
		m_backLeftModule = new SwerveModule(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(4, 0),
				BACK_LEFT_MODULE_DRIVE_MOTOR,
				BACK_LEFT_MODULE_STEER_MOTOR,
				BACK_LEFT_MODULE_STEER_ENCODER,
				BACK_LEFT_MODULE_STEER_OFFSET, 0.5, 
				0, 
				5);
		
		m_frontRightModule = new SwerveModule(
				tab.getLayout("Front-Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(2, 0),
				FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				FRONT_RIGHT_MODULE_STEER_MOTOR,
				FRONT_RIGHT_MODULE_STEER_ENCODER,
				FRONT_RIGHT_MODULE_STEER_OFFSET, 0.5, 
				0, 
				5);

		m_frontLeftModule = new SwerveModule(
				tab.getLayout("Front Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(0, 0),
				FRONT_LEFT_MODULE_DRIVE_MOTOR,
				FRONT_LEFT_MODULE_STEER_MOTOR,
				FRONT_LEFT_MODULE_STEER_ENCODER,
				FRONT_LEFT_MODULE_STEER_OFFSET, 0.5, 
				0, 
				5);
		setRobotRampRate(0.0);
		setRobotIdleMode(IdleMode.kBrake);
		/* prevents a random turn when renabling the robot */
		desiredHeading = getGyroscopeRotation().getDegrees();
	}


	public static SwerveSubsystem getInstance() {
		if(instance == null) {
			instance = new SwerveSubsystem();
		}
		return instance;
	}

	private void setRobotRampRate(double rate) {
		m_frontLeftModule.setRampRate(rate);
		m_frontRightModule.setRampRate(rate);
		m_backLeftModule.setRampRate(rate);
		m_backRightModule.setRampRate(rate);
	}

	public void setRobotIdleMode(IdleMode mode) {
		m_frontLeftModule.setControllerMode(mode);
		m_frontRightModule.setControllerMode(mode);
		m_backLeftModule.setControllerMode(mode);
		m_backRightModule.setControllerMode(mode);
	}

	public static void increaseVoltage() {
		MAX_VOLTAGE+=1;
		if(MAX_VOLTAGE > 5) {
			MAX_VOLTAGE = 5;
		}
	}

	public static void decreaseVoltage() {
		MAX_VOLTAGE-=1;
		if(MAX_VOLTAGE < 1) {
			MAX_VOLTAGE = 1;
		}
	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the
	 * robot is currently facing to the
	 * 'forwards' direction.
	 */
	public void zeroGyroscope() {
		m_navx2.zeroNavHeading();
		desiredHeading = 0;
		m_odometer.resetPosition(getPose(), getGyroscopeRotation());		
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
	 * Set the position of the drivetrain
	 *
	 * @param pose the position of the drivetrain to be set
	 */
	public void setPose(Pose2d pose) {
		m_odometer.setPoseMeters(pose);
	}

	/**
	 * Reset the position of the drivetrain.
	 *
	 * @param pose the current position of the drivetrain
	 */
	public void resetOdometry() {
		m_odometer.resetPosition(getPose(), getGyroscopeRotation());
	}

	public Rotation2d getGyroscopeRotation() {
		return m_navx2.getRotation2d();
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}

	public void setModuleStates(SwerveModuleState[] states) {
		for (SwerveModuleState i : states) {
			if (i.speedMetersPerSecond < 0.001) {
				m_frontLeftModule.set(0, m_frontLeftModule.getSteerAngle());
				m_frontRightModule.set(0, m_frontRightModule.getSteerAngle());
				m_backLeftModule.set(0, m_backLeftModule.getSteerAngle());
				m_backRightModule.set(0, m_backRightModule.getSteerAngle());
				return;
			}
		}
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

		m_frontLeftModule.set((states[0].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND),
			states[0].angle.getRadians());
		m_frontRightModule.set((states[1].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND),
			states[1].angle.getRadians());
		m_backLeftModule.set((states[2].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND),
			states[2].angle.getRadians());
		m_backRightModule.set((states[3].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND),
			states[3].angle.getRadians());
	  }

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Gyro Reading", getGyroscopeRotation().getDegrees());
		m_odometer.update(getGyroscopeRotation(), m_frontLeftModule.getState(), m_frontRightModule.getState(),
		m_backLeftModule.getState(), m_backRightModule.getState());
		SmartDashboard.putString("Odometry", m_odometer.getPoseMeters().toString());

		// Drift correction code
		XY = Math.abs(m_frontLeftModule.getDriveVelocity());
		if (Math.abs(m_navx2.getRate()) > 0.2) {
			desiredHeading = getGyroscopeRotation().getDegrees();
		} else if(XY > 0) {
			double calc = driftCorrectionPID.calculate(getGyroscopeRotation().getDegrees(), desiredHeading);
			if(Math.abs(calc) >= 0.55) {
				m_chassisSpeeds.omegaRadiansPerSecond += calc;
			}
		}
		pXY = XY;

		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

		setModuleStates(states);
	}

	public void stop() {
		m_chassisSpeeds = new ChassisSpeeds();
	}
}
