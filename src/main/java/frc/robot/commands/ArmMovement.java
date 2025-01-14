// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm_Subsystem;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class ArmMovement extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm_Subsystem m_subsystem;
  private double m_LeftStickY;
  private double m_RightStickY;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmMovement(Arm_Subsystem subsystem, double LeftStickY, double RightStickY) {
    m_subsystem = subsystem;
    m_LeftStickY = LeftStickY;
    m_RightStickY = RightStickY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setSpeedArm(m_LeftStickY/10.0);
    m_subsystem.setSpeedTelescope(m_RightStickY/10.0);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
