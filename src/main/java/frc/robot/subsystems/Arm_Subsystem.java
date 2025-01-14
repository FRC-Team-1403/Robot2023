// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
                         
public class Arm_Subsystem extends SubsystemBase {
  final SparkMax m_arm;
  final SparkMax m_telescope;
  
  public Arm_Subsystem() {
    m_arm = new SparkMax(Constants.arm, MotorType.kBrushless);
    m_telescope = new SparkMax(Constants.telescope, MotorType.kBrushless);
  }

  public void setSpeedArm(double speed) {
    m_arm.set(speed);
  }

  public void setSpeedTelescope(double speed) {
    m_telescope.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
