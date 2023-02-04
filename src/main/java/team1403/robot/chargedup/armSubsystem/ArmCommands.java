
package team1403.robot.chargedup.armSubsystem;

import java.util.function.DoubleSupplier;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MoveAction;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.chargedup.RobotConfig;

/**
 * class ArmCommands is the where the commands for Arm.java is located
 */
public class ArmCommands extends CommandBase {
  private final RobotConfig.Arm m_armConfig;
  private DoubleSupplier m_armAngleSupplier;
  private DoubleSupplier m_wristAngleSupplier;
  private DoubleSupplier m_armExtensionIncreaseSupplier;
  private DoubleSupplier m_armExtensionDecreaseSupplier;
  

  private final Arm m_arm;

  private double m_armAngle;
  private double m_wristAngle;
  private double m_armExtension;

  /**
   * defines spinny, as m_arm.
   * sets xBoxControllerInput to controller
   */
  public ArmCommands(Arm spinny, DoubleSupplier m_armAngle, DoubleSupplier m_wristAngle,
      DoubleSupplier m_armExtensionIncrease, DoubleSupplier m_armExtensionDecrease, RobotConfig robotConfig) {
    this.m_arm = spinny;
    this.m_armAngleSupplier = m_armAngle;
    this.m_wristAngleSupplier = m_wristAngle;
    this.m_armExtensionIncreaseSupplier = m_armExtensionIncrease;
    this.m_armExtensionDecreaseSupplier = m_armExtensionDecrease;
    this.m_armConfig = robotConfig.arm;

    addRequirements(spinny); // "Locks the subsystem to the command"
  }

  @Override
  public void execute() {
    m_armAngle += m_armAngleSupplier.getAsDouble() * 360;
    m_armExtension += m_armExtensionIncreaseSupplier.getAsDouble() * m_armConfig.kMaxArmExtension;
    m_armExtension += m_armExtensionDecreaseSupplier.getAsDouble() * m_armConfig.kMaxArmExtension;
    m_wristAngle += m_wristAngleSupplier.getAsDouble() * 360;
    m_arm.moveArm(m_armAngle, m_armExtension, m_wristAngle);
  }

}
