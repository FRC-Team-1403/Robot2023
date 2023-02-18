
package team1403.robot.chargedup.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * class ArmCommands is the where the commands for Arm.java is located
 */
public class ArmCommands extends CommandBase {
  private final DoubleSupplier m_armAngleSupplier;
  private final DoubleSupplier m_wristAngleSupplier;
  private final BooleanSupplier m_armExtensionIncreaseSupplier;
  private final BooleanSupplier m_armExtensionDecreaseSupplier;
  private BooleanSupplier m_wheelIntakeSupplier;

  private final Arm m_arm;

  private Double m_armAngle;
  private Double m_wristAngle;
  private Double m_armExtension;

  /**
   * Defines the constructor for ArmCommands,
   * sets the instant class level vaiables,
   * to new variables.
   *
   * @param arm the Armsubsystem
   * @param armAngle function that determines arm angle, -1 to 1
   * @param wristAngle function that determines wrist angle, -1 to 1
   * @param armExtensionIncrease function that determines
    the increase of arm extension, 0 to 1
   * @param armExtensionDecrease function that determines
    the decrease of arm extension, 0 to 1
   */
  public ArmCommands(Arm arm, DoubleSupplier armAngle, DoubleSupplier wristAngle,
      BooleanSupplier armExtensionIncrease, BooleanSupplier armExtensionDecrease,
        BooleanSupplier wheelIntake) {
    this.m_wheelIntakeSupplier = wheelIntake;
    this.m_arm = arm;
    this.m_armAngleSupplier = armAngle;
    this.m_wristAngleSupplier = wristAngle;
    this.m_armExtensionIncreaseSupplier = armExtensionIncrease;
    this.m_armExtensionDecreaseSupplier = armExtensionDecrease;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    
    super.initialize();

    this.m_armAngle = arm.getArmAngle();
    this.m_wristAngle = arm.getWristAngle();
    this.m_armExtension = arm.getArmExtension();
  }

  @Override
  public void execute() {
    m_armAngle += m_armAngleSupplier.getAsDouble() * 2 /*360*/;

    if (m_armExtensionIncreaseSupplier.getAsBoolean()) {
      m_armExtension += 2;
    }

    if (m_armExtensionDecreaseSupplier.getAsBoolean()) {
      m_armExtension -= 2;
    }
    
    m_wristAngle += m_wristAngleSupplier.getAsDouble() * 2 /*360*/;

    m_arm.moveArm(m_armAngle, m_armExtension, m_wristAngle,
        m_wheelIntakeSupplier.getAsBoolean() ? 1 : 0);
  }

}
