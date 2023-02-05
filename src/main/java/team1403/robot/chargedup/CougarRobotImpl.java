package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarRobot;
import team1403.lib.subsystems.BuiltinSubsystem;
import team1403.lib.util.CougarLogger;
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
    XboxController xboxOperator = getJoystick("Operator", RobotConfig.OperatorConfig.pilotPort);

    m_arm.setDefaultCommand(new ArmCommands(m_arm,
        () -> xboxOperator.getLeftY(),
        () -> xboxOperator.getRightY(),
        () -> xboxOperator.getRightTriggerAxis(),
        () -> xboxOperator.getLeftTriggerAxis()));
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

  private final BuiltinSubsystem m_builtins;
  private final Arm m_arm;
}
