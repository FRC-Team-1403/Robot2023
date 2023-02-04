package team1403.robot.chargedup;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarRobot;
import team1403.lib.subsystems.BuiltinSubsystem;
import team1403.lib.util.CougarLogger;

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
   * @param config Our robot's custom configuration values.
   */
  public CougarRobotImpl(CougarLibInjectedParameters parameters,
                         RobotConfig config) {
    super(parameters);
    var logger = CougarLogger.getChildLogger(
        parameters.getRobotLogger(), "BuiltinDevices");

    m_builtins = new BuiltinSubsystem(parameters, logger);

    m_experimentalSubsystem = new ExperimentalSubsystem("ExperimentalSubsystem", parameters);

    var scheduler = CommandScheduler.getInstance();
    scheduler.registerSubsystem(m_builtins);

    configureOperatorInterface(config.operator);
  }

  /**
   * Configures the operator commands and their bindings.
   */
  private void configureOperatorInterface(
      RobotConfig.OperatorConfig config) {
    // XboxController xboxDriver = getJoystick("Driver", config.pilotPort);

  }

  /**
   * Get controller and silence warnings if not found.
   *
   * @param role The role for the port for logging purposes.
   * @param port The expected port for the controller.
   *
   * @return controller for port, though might not be temporarily disconnected.
   */
  /* private XboxController getJoystick(String role, int port) {
    if (!DriverStation.isJoystickConnected(port)) {
      DriverStation.silenceJoystickConnectionWarning(true);
      CougarLogger.getAlwaysOn().warningf("No controller found on port %d for '%s'",
                                          port, role);
    }
    return new XboxController(port);
  } */

  private final BuiltinSubsystem m_builtins;
  private final ExperimentalSubsystem m_experimentalSubsystem;
}
