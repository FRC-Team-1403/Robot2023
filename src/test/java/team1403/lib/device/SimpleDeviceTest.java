package team1403.lib.device;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import team1403.lib.device.test.FakeMotorController;
import team1403.lib.device.test.MappedDeviceFactory;
import team1403.lib.device.virtual.ManualLimitSwitch;
import team1403.lib.util.CougarLogger;


/**
 * Basic test showing how fake devices can be used to test commands.
 *
 * <p>The command does not know it is using fake devices. There is
 * no "test-only" parts to this command.
 */
class SimpleDeviceTest {
  static final int kLimitSwitchPort = 111;
  static final int kMotorPort = 222;

  /**
   * Defines a simple command speeding up a motor until a limit switch triggers.
   *
   * <p>Note that normally commands interact with subsystems, not
   * directly with devices. The subsystems interact with the devices.
   * The point of this test is to show the linkage from command to
   * the fake devices so we're skipping that layer of abstraction
   * and having the command own the devices.
   */
  class SeekMotorCommand extends CommandBase {
    /**
     * Constructor.
     *
     * @param factory The device factory.
     */
    public SeekMotorCommand(DeviceFactory factory) {
      m_switch = factory.makeLimitSwitch("MyLimitSwitch", kLimitSwitchPort);
      m_motor = factory.makeTalon("MyMotorController", kMotorPort, m_logger);
    }

    /**
     * Returns number of execute() loops this command has run so far.
     */
    public int countExecuteCalls() {
      return m_executeCalls;
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;  // We're in a disabled state when running this unit test.
    }

    @Override
    public void initialize() {
      m_executeCalls = 0;
    }

    @Override
    public boolean isFinished() {
      return m_switch.isTriggered();
    }

    @Override
    public void execute() {
      ++m_executeCalls;
      m_motor.setSpeed(m_executeCalls);
    }

    private int m_executeCalls;
    private final LimitSwitch m_switch;
    private final MotorController m_motor;
  }

  /**
   * Reinitialize our devices and factory before each test.
   */
  @BeforeEach
  void resetTestDevices() {
    m_limitSwitch = new ManualLimitSwitch("MyLimitSwitch");
    m_fakeMotor = new FakeMotorController("MyMotorController", m_logger);

    m_deviceFactory = new MappedDeviceFactory();
    m_deviceFactory.putMotorController(m_fakeMotor);
    m_deviceFactory.putLimitSwitch(m_limitSwitch);
  }


  /**
   * Test how the command and devices interact with one another.
   *
   * <p>We'll run the command to show it controlling the motor,
   * and we'll trigger the limit switch to show how it causes
   * the command to stop executing.
   *
   * <p>Note that this also exposes behavior in the scheduler itself,
   * notably how it only checks whether the command is running after
   * it executes it.
   */  
  @Test
  void testLimitSwitchStopsMotorCommand() {
    SeekMotorCommand command = new SeekMotorCommand(m_deviceFactory);

    assertEquals(0, command.countExecuteCalls());

    // Our fake motor speed defaults to NaN until we explicitly set it.
    // Normally this would be 0, presuming a real motor started off as stopped.
    assertEquals(Double.NaN, m_fakeMotor.getSpeed());
    assertFalse(command.isScheduled());

    var scheduler = CommandScheduler.getInstance();
    scheduler.schedule(command);
    assertTrue(command.isScheduled());
    assertEquals(0, command.countExecuteCalls());
    assertEquals(Double.NaN, m_fakeMotor.getSpeed());

    scheduler.run();
    assertTrue(command.isScheduled());
    assertEquals(1, command.countExecuteCalls());
    assertEquals(1.0, m_fakeMotor.getSpeed());

    scheduler.run();
    assertTrue(command.isScheduled());
    assertEquals(2, command.countExecuteCalls());
    assertEquals(2.0, m_fakeMotor.getSpeed());

    m_limitSwitch.setTriggered(true);
    scheduler.run();
    assertFalse(command.isScheduled());
    assertEquals(3, command.countExecuteCalls());
    assertEquals(3.0, m_fakeMotor.getSpeed());

    for (int i = 0; i < 10; i++) {
      scheduler.run();
      // The command is finished because of the limit switched
      // so got unscheduled.
      assertFalse(command.isScheduled());

      // The command scheduler checks isFinished after execute,
      // so we still did one last execute before we unscheduled.
      // We dont necessarily care about this, but the test is
      // documenting that fact. This will break if the scheduler
      // changes it's implementation, but then we'll just adjust
      // the test. The breakage will keep us informed about that
      // subtle behavior change.
      assertEquals(3, command.countExecuteCalls());
      assertEquals(3.0, m_fakeMotor.getSpeed());
    }

    // Now clear the limit switch and continue running scheduler.
    m_limitSwitch.setTriggered(false);
    for (int i = 0; i < 10; i++) {
      // We never rescheduled so there is no effect,
      // The limit switch did not matter.
      scheduler.run();
      assertFalse(command.isScheduled());
      assertEquals(3, command.countExecuteCalls());
      assertEquals(3.0, m_fakeMotor.getSpeed());
    }

    // schedule the command
    scheduler.schedule(command);
    assertTrue(command.isScheduled());
    // the command initialize cleared the count, but did not touch the motor
    // so the motor is still running.
    assertEquals(0, command.countExecuteCalls());
    assertEquals(3.0, m_fakeMotor.getSpeed());

    // now run scheduler again
    scheduler.run();
    assertTrue(command.isScheduled());
    assertEquals(1, command.countExecuteCalls());
    // The command set the motor (slowing it down from before).
    assertEquals(1.0, m_fakeMotor.getSpeed());

    m_limitSwitch.setTriggered(true);
    scheduler.run();
    assertFalse(command.isScheduled());

    // Schedule the command with the limit switch already triggered
    m_fakeMotor.setSpeed(123.0);
    scheduler.schedule(command);
    assertTrue(command.isScheduled());

    scheduler.run();
    assertFalse(command.isScheduled());
    // Once again, the scheduler had to execute the command once
    // before checking if it was finished.
    assertEquals(1, command.countExecuteCalls());
    assertEquals(1.0, m_fakeMotor.getSpeed());
  }

  private ManualLimitSwitch m_limitSwitch;
  private FakeMotorController m_fakeMotor;
  private MappedDeviceFactory m_deviceFactory;

  private final CougarLogger m_logger = CougarLogger.getLoggerForClass(SimpleDeviceTest.class);
}
