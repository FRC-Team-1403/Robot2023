package team1403.lib.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Function;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import team1403.lib.device.test.MappedDeviceFactory;
import team1403.lib.device.wpi.RealDeviceFactory;
import team1403.lib.util.Clock;
import team1403.lib.util.CougarLogger;
import team1403.lib.util.WpiClock;
import team1403.lib.util.test.FakeClock;


/**
 * WpiLibRobtoAdapter test fixture.
 */
@SuppressWarnings({"PMD.MutableStaticState", "PMD.AssignmentToNonFinalStatic"})
class WpiLibRobotAdapterTest {
  public static TestRobot testRobotCreated;
  public static TestableWpiLibRobotAdapter testAdapterCreated;

  /**
   * This is our custom CougarRobot we want to create.
   */
  class TestRobot extends CougarRobot {
    // The autonomous command to test with.
    Command autonomousCommand;

    /**
     * Constructor.
     *
     * @param parameters CougarRobot framework parameters.
     */
    public TestRobot(CougarLibInjectedParameters parameters) {
      super(parameters);
      testRobotCreated = this;
    }

    @Override
    public Command getAutonomousCommand() {
      return autonomousCommand;
    }
  }

  /**
   * Blocks the system.exit() called within the WPI libraries.
   *
   * <p>SecurityManager is deprecated but there is no other way to do this.
   * If/when security manager goes away then this should be ported to the
   * replacement mechanism. Also won't be needed if WPILib removes their
   * hard-coded exits.
   */
  @SuppressWarnings({"Deprecation", "removal"})
  class DisableExitSecurityManager extends SecurityManager {
    /**
     * Constructor.
     *
     * @param original The existing security manager to delegate to.
     */
    public DisableExitSecurityManager(SecurityManager original) {
      m_originalSecurityManager = original;
    }

    /**
     * Get the trapped exit status.
     *
     * @return null if System.exit() has not been called.
     */
    public Integer getStatusCodeOrNull() {
      return m_statusCode;
    }

    @Override
    public void checkExit(int statusCode) {
      if (m_statusCode == null) {
        m_statusCode = statusCode;
      }
      throw new SecurityException("Blocked exit()");
    }

    @Override
    public void checkPermission(java.security.Permission permission) {
      if (m_originalSecurityManager != null) {
        m_originalSecurityManager.checkPermission(permission);
      }
    }

    private final SecurityManager m_originalSecurityManager;
    private Integer m_statusCode;
  }

  /**
   * A more testable WpiLibRobotAdapter.
   *
   * <p>Will run just enough to construct the TestRobot.
   */
  class TestableWpiLibRobotAdapter extends WpiLibRobotAdapter<TestRobot> {
    //  How many times we called initRobot.
    public int robotInitCount;

    /**
     * Constructor.
     *
     * @param cougarRobotFactory The factory that will build our robot.
     */
    public TestableWpiLibRobotAdapter(
        Function<CougarLibInjectedParameters, TestRobot> cougarRobotFactory) {
      super(cougarRobotFactory);
      m_dataloggingEnabled = false;  // Otherwise future tests will hang.
      testAdapterCreated = this;
    }

    @Override
    public void robotInit() {
      super.robotInit();
      ++robotInitCount;
    }

    /**
     * This is normally implemented by WPI RobotBase.
     *
     * <p>Here we will return after constructing the robot.
     * Normally this is an infinite loop without a means to break out of.
     */
    @Override
    public void startCompetition() {
      robotInit();
    }

    /**
     * Expose the loopFunc so we can test the evaluation cycle.
     *
     * <p>Maybe this should be on the adapter class itself as a
     * loopFuncTestOnly() so we do not need to subclass the adapter only
     * to be able to control the loop execution in other tests.
     */
    @Override
    public void loopFunc() {
      super.loopFunc();
    }
  }

  @Test
  void testDefaultCougarParameters() {
    CougarLibInjectedParameters params = new CougarLibInjectedParameters.Builder().build();
    assertNotNull(params.getRobotLogger());
    assertSame(WpiClock.instance(), params.getClock());
    assertTrue(params.getDeviceFactory() instanceof RealDeviceFactory);
  }

  @Test
  void testConstructor() {
    final var logger = CougarLogger.getCougarLogger("TestLogger");
    final var clock = new FakeClock();
    final var deviceFactory = new MappedDeviceFactory();

    Function<CougarLibInjectedParameters, TestRobot> cougarFactory
        = (CougarLibInjectedParameters params) -> {
          return new TestRobot(params);
        };


    // This try is just to avoid complaint about closing the adapter
    // because it is autoclosable..
    try (var adapter = new TestableWpiLibRobotAdapter(cougarFactory)) {
      // Modify the parameters for the sake of testing the plumbing.
      adapter.setDefaultCougarLibInjectedParameters(
          new CougarLibInjectedParameters.Builder()
          .clock(clock)
          .robotLogger(logger)
          .deviceFactory(deviceFactory)
          .build());

      // robotInit will create the CougarRobot as a side effect.
      assertNull(adapter.getCougarRobot());
      adapter.robotInit();
      CougarRobot robot = adapter.getCougarRobot();
      assertNotNull(robot);

      // Verify mode transitions.
      assertEquals(CougarRobot.Mode.OFFLINE, robot.getMode());
      assertNull(robot.getAutonomousCommand());
      assertEquals(clock, robot.getInjectedParameters().getClock());
      assertEquals(logger, robot.getInjectedParameters().getRobotLogger());
      assertEquals(deviceFactory,
                   robot.getInjectedParameters().getDeviceFactory());

      DriverStationSim.setDsAttached(false);
      DriverStationSim.setAutonomous(true);
      adapter.loopFunc();
      assertEquals(CougarRobot.Mode.DISABLED, robot.getMode());
      DriverStationSim.setDsAttached(true);
      adapter.loopFunc();
      assertEquals(CougarRobot.Mode.AUTONOMOUS, robot.getMode());
      DriverStationSim.setEnabled(false);
      adapter.loopFunc();
      assertEquals(CougarRobot.Mode.DISABLED, robot.getMode());
      DriverStationSim.setEnabled(true);
      adapter.loopFunc();
      assertEquals(CougarRobot.Mode.AUTONOMOUS, robot.getMode());

      DriverStationSim.setAutonomous(false);
      adapter.loopFunc();
      assertEquals(CougarRobot.Mode.TELEOP, robot.getMode());

      DriverStationSim.setTest(true);
      adapter.loopFunc();
      assertEquals(CougarRobot.Mode.TEST, robot.getMode());

      DriverStationSim.setTest(false);
      adapter.loopFunc();
      assertEquals(CougarRobot.Mode.TELEOP, robot.getMode());

      DriverStationSim.setEnabled(false);
      adapter.loopFunc();
      assertEquals(CougarRobot.Mode.DISABLED, robot.getMode());

      // Can repeat modes.
      DriverStationSim.setEnabled(true);
      DriverStationSim.setAutonomous(true);
      adapter.loopFunc();
      assertEquals(CougarRobot.Mode.AUTONOMOUS, robot.getMode());

      // Does not change mode.
      adapter.simulationInit();
      assertEquals(CougarRobot.Mode.AUTONOMOUS, robot.getMode());
    }
  }

  @Test
  void testAutonomousCommand() {
    final var clock = new FakeClock();
    final long startMicros = 12345670;
    clock.advanceSecs(startMicros * Clock.kSecsPerMicro);
    Function<CougarLibInjectedParameters, TestRobot> cougarFactory
        = (CougarLibInjectedParameters params) -> {
          return new TestRobot(params);
        };

    // Our command will advance the clock.
    // This way we can test the integrity of the internal timers.
    // Note that we could use an InstantCommand here, but InstantCommands
    // execute in their initialize() method rather than in their execute()
    // method, which means it would execute when we schedule in autonomousInit()
    // rather than in robotPeriodic() when would would normally execute() commands.
    // We want to test that robotPeriodic is properly using the scheduler.
    // InstantCommands wont do that because they will have already finished.
    final long commandMicros = 500;
    Command autonomousCommand = new Command() {
        @Override
        public Set<Subsystem> getRequirements() {
          return new HashSet<Subsystem>();
        }

        @Override
        public void execute() {
          clock.advanceSecs(commandMicros * Clock.kSecsPerMicro);
        }
    };

    try (var adapter = new TestableWpiLibRobotAdapter(cougarFactory)) {
      assertNull(adapter.getCougarRobot());
      adapter.setDefaultCougarLibInjectedParameters(
          new CougarLibInjectedParameters.Builder()
          .clock(clock)
          .build());
      adapter.robotInit();
      TestRobot robot = adapter.getCougarRobot();
      robot.autonomousCommand = autonomousCommand;

      DriverStationSim.setEnabled(false);
      adapter.loopFunc();
      long periodMicros = 23456;
      clock.advanceSecs(periodMicros * Clock.kSecsPerMicro);

      DriverStationSim.setEnabled(true);
      adapter.loopFunc();
      assertEquals(startMicros + periodMicros, clock.nowMicros());
      DriverStationSim.setAutonomous(true);
      adapter.loopFunc();
      assertEquals(startMicros + commandMicros + periodMicros, clock.nowMicros());
      clock.advanceSecs(periodMicros * Clock.kSecsPerMicro);
      adapter.loopFunc();
      assertEquals(startMicros + 2 * (commandMicros + periodMicros), clock.nowMicros());

      assertTrue(autonomousCommand.isScheduled());
      assertEquals(commandMicros * Clock.kMillisPerMicro,
                   SmartDashboard.getEntry("SchedulerTimer").getNumber(-1));
      assertEquals(periodMicros * Clock.kMillisPerMicro,
                   SmartDashboard.getEntry("LoopPeriodTimer").getNumber(-1));

      // teleopInit cancels the autonomous command if any when leaving autonomous.
      DriverStationSim.setAutonomous(false);
      clock.advanceSecs(periodMicros * Clock.kSecsPerMicro);
      adapter.loopFunc();

      // Command stopped running so clock wasnt advanced.
      assertEquals(0.0, SmartDashboard.getEntry("SchedulerTimer").getNumber(-1));
      assertEquals(periodMicros * Clock.kMillisPerMicro,
                   SmartDashboard.getEntry("LoopPeriodTimer").getNumber(-1));
    }
  }

  @Test
  @SuppressWarnings({"Deprecation", "removal", "PMD.EmptyCatchBlock"})
  void testStartRobot() {
    Function<CougarLibInjectedParameters, TestRobot> cougarFactory =
        (CougarLibInjectedParameters params) -> {
          return new TestRobot(params);
        };

    WpiLibRobotAdapter.suppressExitWarning(true);
    var oldSecurityManager = System.getSecurityManager();
    var disableExitSecurityManager
        = new DisableExitSecurityManager(oldSecurityManager);
    try {
      System.setSecurityManager(disableExitSecurityManager);

      WpiLibRobotAdapter.startRobot(
          () -> {
            return new TestableWpiLibRobotAdapter(cougarFactory);
          });
    } catch (SecurityException ex) {
      // empty
    } finally {
      System.setSecurityManager(oldSecurityManager);
    }

    var statusCode = disableExitSecurityManager.getStatusCodeOrNull();
    assertNotNull(statusCode);
    assertEquals(0, statusCode.intValue());

    assertNotNull(testAdapterCreated);
    assertEquals(1, testAdapterCreated.robotInitCount);
    assertNotNull(testRobotCreated);

    assertEquals(testRobotCreated, testAdapterCreated.getCougarRobot());
  }
}
