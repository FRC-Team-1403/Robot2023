// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot.chargedup.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.ModuleConstants;
import team1403.lib.device.SwerveModule;
import team1403.robot.chargedup.RobotConfig;

import java.lang.reflect.*;

public class Drivetrain extends CougarSubsystem {

	/**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static double MAX_VOLTAGE = 5.0;

	private final AHRS m_navx = new AHRS(SPI.Port.kMXP);
	private SwerveModulePosition[] m_position = new SwerveModulePosition[4];
	private final SwerveDriveOdometry m_odometer = new SwerveDriveOdometry(
		ModuleConstants.kDriveKinematics,
			getGyroscopeRotation(), m_position);
	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;
	private final RobotConfig.SwerveConfig m_config;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	public Drivetrain(CougarLibInjectedParameters injectedParameters, RobotConfig config) {
		super("Drivetrain", injectedParameters);
		m_config = config.swerveConfig;

		m_backRightModule = new SwerveModule("Back Right Module", ModuleConstants.BR_Drive_Id, ModuleConstants.BR_Steer_Id, 
			ModuleConstants.BR_Steer_Id, ModuleConstants.BR_Encoder_Offset, getLogger(), config);
		
		m_backLeftModule = new SwerveModule("Back Left Module", ModuleConstants.BL_Drive_Id, ModuleConstants.BL_Steer_Id, 
			ModuleConstants.BL_Steer_Id, ModuleConstants.BL_Encoder_Offset, getLogger(), config);
		
		m_frontRightModule = new SwerveModule("Front Right Module", ModuleConstants.FR_Drive_Id, ModuleConstants.FR_Steer_Id, 
			ModuleConstants.FR_Steer_Id, ModuleConstants.FR_Encoder_Offset, getLogger(), config);

    m_frontLeftModule = new SwerveModule("Front Left Module", ModuleConstants.FL_Drive_Id, ModuleConstants.FL_Steer_Id, 
			ModuleConstants.FL_Steer_Id, ModuleConstants.FL_Encoder_Offset, getLogger(), config);

		setRobotIdleMode(IdleMode.kCoast);
		
		m_position[0] = m_backRightModule.getModulePosition();
		m_position[1] = m_backLeftModule.getModulePosition();
		m_position[2] = m_frontLeftModule.getModulePosition();
		m_position[3] = m_frontRightModule.getModulePosition();
	}

	public void setRobotIdleMode(IdleMode mode) {
		m_frontLeftModule.setControllerMode(mode);
		m_frontRightModule.setControllerMode(mode);
		m_backLeftModule.setControllerMode(mode);
		m_backRightModule.setControllerMode(mode);
  }

	public static void increaseVoltage() {
		MAX_VOLTAGE++;
		if(MAX_VOLTAGE > 5) {
			MAX_VOLTAGE = 5;
		}
	}

	public static void decreaseVoltage() {
		MAX_VOLTAGE--;
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
		m_navx.zeroYaw();
		m_navx.reset();
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
	 * @throws IllegalAccessException
	 * @throws IllegalArgumentException
	 */
	public void setPose(Pose2d pose) throws IllegalArgumentException, IllegalAccessException {
		Field[] fields = m_odometer.getClass().getFields();
		for(Field field : fields)
		{
			if(field.getName() == "m_poseMeters")
			{
				field.set(m_odometer, pose);
			}
		}
	}


	/**
	 * Reset the position of the drivetrain.
	 *
	 * @param pose the current position of the drivetrain
	 */
	public void resetOdometry() {
		m_odometer.resetPosition(getGyroscopeRotation(), m_position, getPose());
	}

	public Rotation2d getGyroscopeRotation() {
		if (m_navx.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(-m_navx.getFusedHeading());
		}
		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
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
			}
		}

		m_frontLeftModule.set((states[0].speedMetersPerSecond / ModuleConstants.kMaxSpeed * MAX_VOLTAGE),
			states[0].angle.getRadians());
		m_frontRightModule.set((states[1].speedMetersPerSecond / ModuleConstants.kMaxSpeed * MAX_VOLTAGE),
			states[1].angle.getRadians());
		m_backLeftModule.set((states[2].speedMetersPerSecond / ModuleConstants.kMaxSpeed * MAX_VOLTAGE),
			states[2].angle.getRadians());
		m_backRightModule.set((states[3].speedMetersPerSecond / ModuleConstants.kMaxSpeed * MAX_VOLTAGE),
			states[3].angle.getRadians());
	  }

	@Override
	public void periodic() {
		SwerveModuleState[] states = ModuleConstants.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);
		setModuleStates(states);
		m_position[0] = m_backRightModule.getModulePosition();
		m_position[1] = m_backLeftModule.getModulePosition();
		m_position[2] = m_frontLeftModule.getModulePosition();
		m_position[3] = m_frontRightModule.getModulePosition();
	}

	public void stop() {
		m_chassisSpeeds = new ChassisSpeeds();
	}
}