package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarRobot;
import team1403.lib.subsystems.BuiltinSubsystem;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.RobotConfig.OperatorConfig;
import team1403.robot.chargedup.arm.Arm;
import team1403.robot.chargedup.arm.ArmCommands;

/**
 * The heart of the robot.
 *
 * <p>The bulk of the robot will be implemented through various subsystems.
 * This class creates those subsystems and configures their behaviors.
 *
 * <p>This class has little to do with the runtime operation. It acts more as
 * a factory to create the subsystems and write them together and to external
 * controls (both human and software). Once that has happened, the controls
 * take over and issue commands that interact with the subsystem to actually
 * do things.
 */
public class CougarRobotImpl extends CougarRobot {

  /**
   * Constructor.
   *
   * @param parameters Standard framework injected parameters.
   */
  public CougarRobotImpl(CougarLibInjectedParameters parameters) {
    super(parameters);

    var logger = CougarLogger.getChildLogger(
        parameters.getRobotLogger(), "BuiltinDevices");

    m_builtins = new BuiltinSubsystem(parameters, logger);
    m_arm = new Arm(parameters);

    var scheduler = CommandScheduler.getInstance();
    scheduler.registerSubsystem(m_builtins);

    configureOperatorInterface();
  }

  /**
   * Configures the operator commands and their bindings.
   */
  private void configureOperatorInterface() {
    XboxController xboxOperator = getJoystick("Operator", OperatorConfig.pilotPort);

    new Trigger(() -> xboxOperator.getYButton()).onFalse(
        new InstantCommand(() -> switchOperatorMode()));
    
    if (m_armOperatorManual) {
      manualOperatorMode(xboxOperator);
    } else {
      autoOperatorMode(xboxOperator);
    }
  }

  /**
   * Get controller and silence warnings if not found.
   *
   * @param role The role for the port for logging purposes.
   * @param port The expected port for the controller.
   *
   * @return controller for port, though might not be temporarily disconnected.
   */
  private XboxController getJoystick(String role, int port) {
    if (!DriverStation.isJoystickConnected(port)) {
      DriverStation.silenceJoystickConnectionWarning(true);
      CougarLogger.getAlwaysOn().warningf("No controller found on port %d for '%s'",
          port, role);
    }
    return new XboxController(port);
  }

  //TODO, figure out actual setpoint values
  
  /**
   * This is the auto mode for operator.
   * Has 5 setpoints, which will each set the arm
   * in different positions
   * A Button -> Ground
   * B Button -> Shelf
   * Dpad down Button -> Tuck
   * DPad Up -> High
   * DPad Right -> Mid
   *
   * @param xboxOperator defines which controller is being used
   */
  public void autoOperatorMode(XboxController xboxOperator) {
    new Trigger(() -> xboxOperator.getAButton()).onFalse(
      new InstantCommand(() -> m_arm.moveArm(0, 0, 0, 0)));
    new Trigger(() -> xboxOperator.getBButton()).onFalse(
        new InstantCommand(() -> m_arm.moveArm(0, 0, 0, 0)));
    new Trigger(() -> xboxOperator.getRawButton(OperatorConfig.dPadDown)).onFalse(
        new InstantCommand(() -> m_arm.moveArm(0, 0, 0, 0)));
    new Trigger(() -> xboxOperator.getRawButton(OperatorConfig.dPadUp)).onFalse(
      new InstantCommand(() -> m_arm.moveArm(0, 0, 0, 0)));
    new Trigger(() -> xboxOperator.getRawButton(OperatorConfig.dPadRight)).onFalse(
        new InstantCommand(() -> m_arm.moveArm(0, 0, 0, 0)));
  }

  /**
   * This is the manual mode for operator.
   * Minutely control arm with joysticks
   * 
   * @param xboxOperator defines which controller is being used
   */
  private void manualOperatorMode(XboxController xboxOperator) {
    m_arm.setDefaultCommand(new ArmCommands(m_arm,
        () -> xboxOperator.getLeftY(),
        () -> xboxOperator.getRightY(),
        () -> xboxOperator.getRightTriggerAxis(),
        () -> xboxOperator.getLeftTriggerAxis()));

  }

  /**
   * Switches the operator mode.
   */
  public void switchOperatorMode() {
    m_armOperatorManual = !m_armOperatorManual;
  }

  private final BuiltinSubsystem m_builtins;
  private final Arm m_arm;
  private boolean m_armOperatorManual = true;
}
