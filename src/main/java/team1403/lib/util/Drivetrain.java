// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.lib.util;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.ModuleConstants;
import team1403.lib.device.SwerveModule;
import team1403.lib.util.NavXGyro;

public class Drivetrain extends SubsystemBase {

	    /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static double MAX_VOLTAGE = 5.0;

	private final AHRS m_navx = new AHRS(SPI.Port.kMXP);
	private final SwerveDriveOdometry m_odometer = new SwerveDriveOdometry(
		ModuleConstants.kDriveKinematics,
			getGyroscopeRotation());

	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);


	private static Drivetrain instance = null;

	private Drivetrain() {
		m_backRightModule = new SwerveModule(ModuleConstants.BR_Drive_Id, ModuleConstants.BR_Steer_Id, ModuleConstants.BR_Steer_Id, ModuleConstants.BR_Encoder_Offset);
		m_backLeftModule = new SwerveModule(ModuleConstants.BL_Drive_Id, ModuleConstants.BL_Steer_Id, ModuleConstants.BL_Steer_Id, ModuleConstants.BL_Encoder_Offset);
		m_frontRightModule = new SwerveModule(ModuleConstants.FR_Drive_Id, ModuleConstants.FR_Steer_Id, ModuleConstants.FR_Steer_Id, ModuleConstants.FR_Encoder_Offset);
        m_frontLeftModule = new SwerveModule(ModuleConstants.FL_Drive_Id, ModuleConstants.FL_Steer_Id, ModuleConstants.FL_Steer_Id, ModuleConstants.FL_Encoder_Offset);
		setRobotIdleMode(IdleMode.kCoast);
	}

	public static Drivetrain getInstance () {
		if(instance == null) {
			instance = new Drivetrain();
		}
		return instance;
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
			if (i.speedMetersPerSecond < 0.003) {
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
		
	}

	public void stop() {
		m_chassisSpeeds = new ChassisSpeeds();
	}
}
