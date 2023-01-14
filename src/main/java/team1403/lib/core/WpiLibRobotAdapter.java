package team1403.lib.core;

import java.io.File;
import java.util.function.Function;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import team1403.lib.util.BaseTimer;
import team1403.lib.util.Clock;
import team1403.lib.util.CougarLogger;


/**
 * Adapts CougarRobot to the WPILib's BaseRobot framework.
 *
 * <p>In a WPILib robot, there is a "Robot" class that users provide to
 * implement their robot. In practice that class has lots of boilerplate code
 * and is based on worrying about different operating "modes". Often the
 * interesting parts of the given robot are implemented elsewhere. The common
 * pattern is to have a class "RobotContainer" that creates the subsystems and
 * wires in all the sensors and actuators. When using command-based robots
 * (as we do) the "RobotContainer" also sets up the commands and their triggers.
 *
 * <p>This class provides that "Robot" class. It is likely that most of our
 * robots will be doing the same things in this class. Rather than having each
 * robot reimplement the class following our standard patterns, we implement
 * them here.
 *
 * <p>This class is designed to work with a complementary class that provides
 * the implementation of our custom "CougarRobot" classes. These robots are
 * derived from CougarRobot which provides our own * framework around
 * controlling the custom parts of the robot that we build, leaving this base
 * class with the responsibilities of integrating with the WPI libraries that
 * provide the runtime platform integrating with the external environment.
 *
 * <p>The WpiLibRobotAdapter is paired with the custom CougarRobot through
 * Java Generic template. 
 *
 * <table>
 * <caption>Comparison between Wpi RobotBase and WpiLibRobotAdapter/CougarRobot.
 * </caption>
 * <tr><th style="text-align:left">Traditional WPI
 *     <th style="text-align:left">CougarLib
 * <tr><td>RobotBase      <td>WpiLibRobotAdapter&lt;&gt;
 * <tr><td>Robot extends RobotBase<td>WpiLibRobotAdapter&lt;CougarRobot&gt;
 * <tr><td>RobotContainer<td>CougarRobot
 * <tr><td style="vertical-align:top">
 *     <pre>RobotBase.startRobot(Robot::new);</pre>
 *     <td style="vertical-align:top">
 * <pre>Function&lt;CougarLibInjectedParameters, CougarRobot&gt; cougarFactory
 *    = (CougarLibInjectedParameters params)-> { return new CougarRobot(params); };
 * RobotBase.startRobot(
 *   () -> { return new WpiLibRobotAdapter&lt;CougarRobot&gt;(cougarFactory); });
 * </pre></table>
 *
 * <p>This class is constructed [indirectly] by Main. There is no need to
 * explicitly instantiate it other than in tests. This will already be setup for
 * you when starting with the template cougar robot project.
 */
@SuppressWarnings({"PMD.CommentSize", "PMD.TooManyMethods"})
public class WpiLibRobotAdapter<T extends CougarRobot> extends TimedRobot {
  /**
   * Constructor.
   *
   * @param cougarRobotFactory Function that creates a new CougarRobot from
   *        {@link CougarLibInjectedParameters}.
   */
  public WpiLibRobotAdapter(
      Function<CougarLibInjectedParameters, T> cougarRobotFactory) {
    m_cougarRobotFactory = cougarRobotFactory;
  }

  /**
   * Returns the adapted CougarRobot.
   *
   * <p>This call is intended for testing.
   *
   * @return CougarRobot is only available after robotInit has been called.
   */
  public T getCougarRobot() {
    return m_cougarRobot;
  }

  @Override
  public void robotInit() {
    initLogging();
    m_logger = CougarLogger.getLoggerForClass(getClass());
    m_logger.tracef("robotInit");

    // Instantiate our cougar robot.
    CougarLibInjectedParameters.Builder parameterBuilder;
    if (m_defaultInjectedParameters != null) {
      parameterBuilder = T.newParameterBuilder(m_defaultInjectedParameters);
    } else {
      parameterBuilder = T.newParameterBuilder();
      parameterBuilder.robotLogger(CougarLogger.getCougarLogger("team1403.Robot"));
    }
    var parameters = parameterBuilder.build();
    Clock clock = parameters.getClock();
    m_cougarRobot = m_cougarRobotFactory.apply(parameters);
    m_schedulerTimer = new BaseTimer("SchedulerTimer", clock);
    m_periodTimer = new BaseTimer("LoopPeriodTimer", clock);
  }


  /**
   * Manages the work for each loop iteration.
   */
  @Override
  public void robotPeriodic() {
    // Record time between calls
    m_periodTimer.recordMillis();

    // Record time to run all the scheduled commands.
    m_schedulerTimer.timeCallMicros(true,
                                    () -> CommandScheduler.getInstance().run());
    m_periodTimer.restart();  // to capture time to next call
  }

  /**
   * Signals the CougarRobot that we are entering DISABLED mode.
   */
  @Override
  public void disabledInit() {
    m_logger.tracef("disabledInit");
    m_cougarRobot.changeMode(CougarRobot.Mode.DISABLED);
  }

  /**
   * Signals the CougarRobot that we are entering AUTONOMOUS mode.
   */
  @Override
  public void autonomousInit() {
    m_logger.tracef("autonomousInit");
    m_cougarRobot.changeMode(CougarRobot.Mode.AUTONOMOUS);
    m_autonomousCommand = m_cougarRobot.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * Signals the CougarRobot that we are entering TELEOP mode.
   */
  @Override
  public void teleopInit() {
    m_logger.tracef("teleopInit");
    if (m_autonomousCommand != null) {
      // This is done in the WPILib examples so we're leaving it here.
      // But it probably does not make sense to force. If we have a smart
      // autonomous command, why interrupt it if it is still going? Operator
      // can take control to interrupt it.
      m_logger.debugf("Cancelling autonomous command when entering teleop.");
      m_autonomousCommand.cancel();
    }
    m_cougarRobot.changeMode(CougarRobot.Mode.TELEOP);
  }

  /**
   * Signals the CougarRobot that we are entering TEST mode.
   */
  @Override
  public void testInit() {
    m_logger.tracef("testInit");

    m_logger.debugf("Cancelling all commands when entering test");
    CommandScheduler.getInstance().cancelAll();

    m_cougarRobot.changeMode(CougarRobot.Mode.TEST);
  }

  /**
   * Signals the CougarRobot that we are starting to simulate.
   */
  @Override
  public void simulationInit() {
    m_logger.tracef("simulationInit");
    // For now we do not care about this.
    // All this is doing is telling us that the simulation has started.
    // But I don't think we care about that at this time. When we do then
    // we can extend the CougarRobot class with another method to
    // call here.
  }

  @Override
  public void disabledPeriodic() {
    // We dont want but base class forces us to override!
  }

  @Override
  public void autonomousPeriodic() {
    // We dont want but base class forces us to override!
  }

  @Override
  public void teleopPeriodic() {
    // We dont want but base class forces us to override!
  }

  @Override
  public void testPeriodic() {
    // We dont want but base class forces us to override!
  }

  @Override
  public void simulationPeriodic() {
    // We dont want but base class forces us to override!
  }


  /**
   * Sets the default robot parameters to initialize newCougarLibInjectedParameterBuilder
   *
   * <p>This is only intended to provide tests a means to inject values.
   * Normally the class provides the defaults to use which permits new custom
   * adapters to add additional parameters if needed, but not to initialize their
   * default values. Since these are created and used internally, tests would not
   * otherwise have access to intercept these to inject desired values.
   *
   * @param params The desired parameter values.
   */
  public void setDefaultCougarLibInjectedParameters(CougarLibInjectedParameters params) {
    m_defaultInjectedParameters = params;
  }

  /**
   * Initialize debug and data logging.
   */
  private void initLogging() {
    // Start data logging to Logs/ directory
    // The operating directory will be /home/lvuser on rio or pwd in simulator.
    File operatingDirectory = Filesystem.getOperatingDirectory();
    File logDir = new File(operatingDirectory, "Logs");

    logDir.mkdir();              // create if it does not already exist.
    logDir.setWritable(true);    // so we can write logs.
    logDir.setReadable(true);    // so we can read the logs afterward.
    logDir.setExecutable(true);  // so we can list the directory afterward.

    System.err.println("Logging to " + logDir);  // NOPMD

    File deployDirectory = Filesystem.getDeployDirectory();
    if (!isReal()) {
      // If running from a workspace, use the source directory itself.
      String path = String.format("%s/Robot/src/main/deploy",
                                  Filesystem.getOperatingDirectory().getPath());
      deployDirectory = new File(path);
    }
    CougarLogger.initModule(
        new File(deployDirectory, "logging.properties").toPath(),
        new File(logDir, CougarLogger.kLogFileBaseName + ".log").toPath());
    if (m_dataloggingEnabled) {
      DataLogManager.start(logDir.toString(),
                           CougarLogger.kLogFileBaseName + ".wpilog");
    }
  }

  /**
   * The factory that will create the custom CougarRobot.
   *
   * <p>This is injected directly into the constructor and used in robotInit.
   */
  private final Function<CougarLibInjectedParameters, T> m_cougarRobotFactory;

  /**
   * Meter for recording how long we spend executing commands.
   */
  private BaseTimer m_schedulerTimer;

  /**
   * Meter for recording time interval between robotPeriodic calls.
   *
   * <p>We'll record the interval from the start of one to the start of the next
   * to see how timely and consistent we are.
   */
  private BaseTimer m_periodTimer;

  /**
   * The logger we'll use for this adapter.
   */
  private CougarLogger m_logger;

  /**
   * Remember the autonomous command so we can cancel it after authonomous mode.
   *
   * <p>This is discovered in autonomousInit.
   */
  private Command m_autonomousCommand;

  /**
   * The CougarRobot managed by this controller.
   *
   * <p>This will be created within robotInit.
   */
  private T m_cougarRobot;

  /**
   * Overriden default cougar robot parameters.
   *
   * <p>If set, then use these as the default values when creating the
   * CougarLibInjectedParameters.Builder. This only exists so that tests
   * can inject specific parameters because they cannot otherwise
   * create custom inner classes and override the newParameterBuilder
   * method because it is static and Java forbids it.
   */
  private CougarLibInjectedParameters m_defaultInjectedParameters;

  /**
   * DataLogManager cannot be turned off once it is started.
   *
   * <p>This gets in the way of test suites so allow unit tests to turn it off.
   */
  protected boolean m_dataloggingEnabled = true;
}
