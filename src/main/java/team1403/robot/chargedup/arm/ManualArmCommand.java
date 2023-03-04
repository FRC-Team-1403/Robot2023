
package team1403.robot.chargedup.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * class ArmCommands is the where the commands for Arm.java is located
 */
public class ManualArmCommand extends CommandBase {
  private final DoubleSupplier m_armAngleSupplier;
  private final DoubleSupplier m_wristAngleSupplier;
  private final DoubleSupplier m_armExtensionDecreaseSupplier;
  private final DoubleSupplier m_armExtensionIncreaseSupplier;
  private final BooleanSupplier m_wheelIntakeSupplier;
  private final BooleanSupplier m_wheelOuttakeSupplier;


  private final Arm_Subsystem m_arm;

  private double m_pivotAngle;
  private double m_wristAngle;
  private double m_armExtension;

  /**
   * Defines the constructor for ArmCommands,
   * sets the instant class level vaiables,
   * to new variables.
   *
   * @param arm the Armsubsystem
   * @param armAngle function that determines arm angle, -1 to 1
   * @param wristAngle function that determines wrist angle, -1 to 1
   * @param armExtensionDecrease function that determines
    the increase of arm extension, 0 to 1
    the decrease of arm extension, 0 to 1
   */
  public ManualArmCommand(Arm_Subsystem arm, DoubleSupplier armAngle, DoubleSupplier wristAngle,
      DoubleSupplier armExtensionDecrease, DoubleSupplier armExtensionIncrease,
        BooleanSupplier wheelIntake, BooleanSupplier wheelOuttake) {
    this.m_wheelIntakeSupplier = wheelIntake;
    this.m_arm = arm;
    this.m_armAngleSupplier = armAngle;
    this.m_wristAngleSupplier = wristAngle;
    this.m_armExtensionDecreaseSupplier = armExtensionDecrease;
    this.m_armExtensionIncreaseSupplier = armExtensionIncrease;
    this.m_wheelOuttakeSupplier = wheelOuttake;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    this.m_pivotAngle = m_arm.getAbsolutePivotAngle();
    this.m_wristAngle = m_arm.getWristAbsoluteAngle();
    this.m_armExtension = m_arm.getExtensionLength();
    System.out.println("Angle: " + m_wristAngle);
    super.initialize();
  }

  @Override
  public void execute() {
    m_wristAngle += m_wristAngleSupplier.getAsDouble() * 4;
    m_wristAngle = m_arm.limitWristAngle(m_wristAngle);

    m_pivotAngle += (-1 * m_armAngleSupplier.getAsDouble());
    m_pivotAngle = m_arm.limitPivotAngle(m_pivotAngle);

    if (m_armExtensionDecreaseSupplier.getAsDouble() > 0) {
      m_armExtension -= m_armExtensionDecreaseSupplier.getAsDouble() / 2;
    } else if (m_armExtensionIncreaseSupplier.getAsDouble() > 0) {
      m_armExtension += m_armExtensionIncreaseSupplier.getAsDouble() / 2;
    }

    m_armExtension = m_arm.limitExtensionLength(m_armExtension);

    SmartDashboard.putNumber("Arm Extension Setpoint", m_armExtension);

    if (this.m_wheelIntakeSupplier.getAsBoolean()) {
      m_arm.moveArm(m_wristAngle, -0.75, m_pivotAngle, m_armExtension);
    } else if (this.m_wheelOuttakeSupplier.getAsBoolean()) {
      m_arm.moveArm(m_wristAngle, 0.75, m_pivotAngle, m_armExtension);
    } else {
      m_arm.moveArm(m_wristAngle, 0, m_pivotAngle, m_armExtension);
    }


    
  }

}
