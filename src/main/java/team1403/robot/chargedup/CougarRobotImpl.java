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
import team1403.robot.chargedup.swerve.SwerveCommand;
import team1403.robot.chargedup.swerve.SwerveSubsystem;

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
    m_swerveSubsystem = new SwerveSubsystem(parameters);

    var scheduler = CommandScheduler.getInstance();
    scheduler.registerSubsystem(m_builtins);

    configureOperatorInterface();
    configureDriverInterface();

  }

  /**
   * Configures the operator commands and their bindings.
   */
  private void configureOperatorInterface() {
    XboxController xboxOperator = getJoystick("Operator", RobotConfig.OperatorConfig.pilotPort);
  }

  /**
   * Configures the driver commands and their bindings.
   */
  private void configureDriverInterface() {
    XboxController xboxDriver = getJoystick("Driver", RobotConfig.DriverConfig.pilotPort);

    new SwerveCommand(
        m_swerveSubsystem,
        () -> -deadband(xboxDriver.getLeftY(), 0.05),
        () -> -deadband(xboxDriver.getLeftX(), 0.05),
        () -> -deadband(xboxDriver.getRightX(), 0.05),
        () -> xboxDriver.getYButtonReleased());

    new Trigger(() -> xboxDriver.getRightBumper()).onFalse(
        new InstantCommand(() -> m_swerveSubsystem.increaseSpeed(0.2)));

    new Trigger(() -> xboxDriver.getLeftBumper()).onFalse(
        new InstantCommand(() -> m_swerveSubsystem.decreaseSpeed(0.2)));
  }

  /**
   * Applies a deadband to the given value.
   *
   * @param value the value to apply a deadband to
   * @param deadband the deadband to apply to the value
   * @return 0 if the value is < deadband,
   *         or value if value > deadband
   */
  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
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

  private final BuiltinSubsystem m_builtins;
  private final SwerveSubsystem m_swerveSubsystem;
}
