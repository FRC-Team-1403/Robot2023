package team1403.robot.__replaceme__;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarRobot;
import team1403.lib.subsystems.BuiltinSubsystem;
import team1403.lib.util.CougarLogger;
import team1403.robot.__replaceme__.examplerail.ExampleRail;
import team1403.robot.__replaceme__.examplerail.SeekCenterCommand;
import team1403.robot.__replaceme__.examplerail.SeekEndCommand;

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

    m_exampleRail = new ExampleRail(parameters, config);
    m_builtins = new BuiltinSubsystem(parameters, logger);

    var scheduler = CommandScheduler.getInstance();
    scheduler.registerSubsystem(m_builtins);
    scheduler.registerSubsystem(m_exampleRail);

    m_autoCommand = new SeekEndCommand(m_exampleRail,
                                       SeekEndCommand.Position.FRONT);

    configureOperatorInterface(config.operator);
  }

  /**
   * Configures the operator commands and their bindings.
   */
  private void configureOperatorInterface(
      RobotConfig.OperatorConfig config) {
    XboxController xboxDriver = getJoystick("Driver", config.pilotPort);

    SeekEndCommand railForward
        = new SeekEndCommand(m_exampleRail, SeekEndCommand.Position.FRONT);
    SeekEndCommand railBackward
        = new SeekEndCommand(m_exampleRail, SeekEndCommand.Position.BACK);
    SeekCenterCommand railCenter
        = new SeekCenterCommand(m_exampleRail, config.seekCenterTolerance);

    new JoystickButton(xboxDriver, Button.kA.value).whenPressed(railForward);
    new JoystickButton(xboxDriver, Button.kY.value).whenPressed(railBackward);
    new JoystickButton(xboxDriver, Button.kX.value).whenPressed(railCenter);

    // Only for the sake of running this without a controller as an example.
    SmartDashboard.putData(railForward);
    SmartDashboard.putData(railCenter);
    SmartDashboard.putData(railBackward);
  }

  /**
   * Get the ExampleRail subsystem.
   *
   * <p>This might be useful for tests but is otherwise not necessary.
   * The robot should inject the subsystem into the constructors for objects
   * that depend on it.
   *
   * @return the ExampleRail subsystem.
   */
  public ExampleRail getExampleRail() {
    return m_exampleRail;
  }

  /**
   * Provides the command used for autonomous.
   *
   * <p>This is special because it does not have a trigger.
   * This function allows the Robot to get it when it enters AUTONOMOUS mode.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {
    return m_autoCommand;
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

  private final ExampleRail m_exampleRail;
  private final BuiltinSubsystem m_builtins;
  private final Command m_autoCommand;
}
