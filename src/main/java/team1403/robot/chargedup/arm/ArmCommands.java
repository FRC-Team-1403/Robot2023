
package team1403.robot.chargedup.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import team1403.robot.chargedup.RobotConfig;

/**
 * class ArmCommands is the where the commands for Arm.java is located
 */
public class ArmCommands extends CommandBase {
  private DoubleSupplier m_armAngleSupplier;
  private DoubleSupplier m_wristAngleSupplier;
  private DoubleSupplier m_armExtensionIncreaseSupplier;
  private DoubleSupplier m_armExtensionDecreaseSupplier;
  private BooleanSupplier m_wheelIntakeSupplier;

  private final Arm m_arm;

  private double m_armAngle;
  private double m_wristAngle;
  private double m_armExtension;

  /**
   * Defines spinny, as m_arm.
   * sets xBoxControllerInput to controller
   */
  public ArmCommands(Arm spinny, DoubleSupplier armAngle, DoubleSupplier wristAngle,
      DoubleSupplier armExtensionIncrease, DoubleSupplier armExtensionDecrease) {
    this.m_arm = spinny;
    this.m_armAngleSupplier = armAngle;
    this.m_wristAngleSupplier = wristAngle;
    this.m_armExtensionIncreaseSupplier = armExtensionIncrease;
    this.m_armExtensionDecreaseSupplier = armExtensionDecrease;

    addRequirements(spinny); // "Locks the subsystem to the command"
  }

  @Override
  public void execute() {
    m_armAngle += m_armAngleSupplier.getAsDouble() * 360;
    m_armExtension += m_armExtensionIncreaseSupplier.getAsDouble()
      * RobotConfig.Arm.kMaxArmExtension;
    m_armExtension += m_armExtensionDecreaseSupplier.getAsDouble()
      * RobotConfig.Arm.kMaxArmExtension;
    m_wristAngle += m_wristAngleSupplier.getAsDouble() * 360;

    m_arm.moveArm(m_armAngle, m_armExtension, m_wristAngle,
        m_wheelIntakeSupplier.getAsBoolean() ? 1 : 0);
  }

}
