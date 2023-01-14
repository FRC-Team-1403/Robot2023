package team1403.robot.__replaceme__;

import static edu.wpi.first.wpilibj.XboxController.Button;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.test.MappedDeviceFactory;
import team1403.lib.subsystems.test.FakeBuiltinSubsystemParts;
import team1403.robot.__replaceme__.examplerail.ExampleRailTest;
import team1403.robot.__replaceme__.examplerail.SeekEndCommand;


/**
 * Test fixture for CougarRobotImpl
 */
class CougarRobotImplTest {

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
    // Enable the driver station so we can run commands
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
  }

  /**
   * Test we construct the robot properly from the injected parameters.
   *
   * <p>The other tests will also test this indirectly, but this will
   * help debug the other tests should they fail.
   */
  @Test
  void testConstructor() {
    final var robotParts = new FakeParts();
    final var parameters = new CougarLibInjectedParameters.Builder()
        .deviceFactory(robotParts.makeFactory())
        .build();
    final var robot = new CougarRobotImpl(parameters, new RobotConfig());

    assertTrue(robot.getAutonomousCommand() instanceof SeekEndCommand);
    final var rail = robot.getExampleRail();
    assertFalse(rail.isAtFront());
    assertFalse(rail.isAtBack());

    final var railParts = robotParts.railParts;

    // Test motor hookup.
    assertEquals(Double.NaN, railParts.fakeMotor.getSpeed());
    rail.setSpeed(0.25);
    assertEquals(0.25, railParts.fakeMotor.getSpeed());

    // Test front Limit switch hookup.
    railParts.frontLimitSwitch.setTriggered(true);
    assertTrue(rail.isAtFront());
    assertFalse(rail.isAtBack());

    // Test back Limit switch hookup.
    railParts.frontLimitSwitch.setTriggered(false);
    railParts.backLimitSwitch.setTriggered(true);
    assertFalse(rail.isAtFront());
    assertTrue(rail.isAtBack());
  }

  @Test
  void testAutonomousCommand() {
    final double kRailSpeed = 0.123;
    final var robotParts = new FakeParts();
    robotParts.robotConfig.exampleRail.motorSpeed = kRailSpeed;

    final var parameters = new CougarLibInjectedParameters.Builder()
        .deviceFactory(robotParts.makeFactory())
        .build();
    final var robot = new CougarRobotImpl(parameters, robotParts.robotConfig);
    final var scheduler = CommandScheduler.getInstance();
    for (int i = 0; i < 5; ++i) {
      scheduler.run();
      assertEquals(Double.NaN, robotParts.railParts.fakeMotor.getSpeed());
    }

    final var command = robot.getAutonomousCommand();
    assertTrue(command instanceof SeekEndCommand);
    assertEquals(SeekEndCommand.Position.FRONT, ((SeekEndCommand)command).getGoal());

    // Since we know this is a FRONT seek, we dont really need to test it
    // because we've already tested SeekEndCommand works.
    // And we tested the base WpiLibRobotAdapter will schedule and run
    // whatever autonomous command we return.


    // We'll test the command anyway for purposes of illustration.
    scheduler.schedule(command);

    for (int i = 0; i < 5; ++i) {
      scheduler.run();
      assertEquals(-kRailSpeed, robotParts.railParts.fakeMotor.getSpeed());
    }

    // Ignores back limit swtich.
    robotParts.railParts.backLimitSwitch.setTriggered(true);
    for (int i = 0; i < 3; ++i) {
      scheduler.run();
    }

    // Stops with limit switch.
    robotParts.railParts.frontLimitSwitch.setTriggered(true);
    scheduler.run();
    assertEquals(0.0, robotParts.railParts.fakeMotor.getSpeed());
  }

  /**
   * Test the autonomous behavior of our robot.
   */
  @Test
  void testOperatorInterface() {
    // Override config for this test with values we'll expect.
    final double kRailSpeed = 0.345;
    final int kPilotPort = 3;
    final var robotParts = new FakeParts();
    robotParts.robotConfig.operator.pilotPort = kPilotPort;
    robotParts.robotConfig.exampleRail.motorSpeed = kRailSpeed;

    // Create the robot injecting a factory using our robot parts.
    final var parameters = new CougarLibInjectedParameters.Builder()
        .deviceFactory(robotParts.makeFactory())
        .build();

    // We dont need the actual robot.
    // To emphasize this, we are not going to assign it to a variable.
    // Constructing the robot has a side effect of setting up the
    // operator interface (binding commands and buttons).
    // We're going to be testing that side effect.
    new CougarRobotImpl(parameters, robotParts.robotConfig);

    final var scheduler = CommandScheduler.getInstance();

    DriverStationSim.setAutonomous(false);   // teleop mode
    DriverStationSim.setJoystickButtonCount(
        kPilotPort, 10);

    // Prime the scheduler and show we arent doing anything yet.
    scheduler.run();
    assertEquals(Double.NaN, robotParts.railParts.fakeMotor.getSpeed());

    // Press the A button to move forward.
    DriverStationSim.setJoystickButton(kPilotPort, Button.kA.value, true);
    DriverStationSim.notifyNewData();
    scheduler.run();
    assertEquals(-kRailSpeed, robotParts.railParts.fakeMotor.getSpeed());
    DriverStationSim.setJoystickButton(kPilotPort, Button.kA.value, false);
    DriverStationSim.notifyNewData();
    // Release Y and continues to move forward.
    scheduler.run();
    assertEquals(-kRailSpeed, robotParts.railParts.fakeMotor.getSpeed());

    // Press A which interrupts forward and starts moving backward
    DriverStationSim.setJoystickButton(kPilotPort, Button.kY.value, true);
    DriverStationSim.notifyNewData();
    scheduler.run();
    assertEquals(kRailSpeed, robotParts.railParts.fakeMotor.getSpeed());

    // Hit rear limit switch to stop command from running.
    robotParts.railParts.backLimitSwitch.setTriggered(true);
    scheduler.run();
    assertEquals(0.0, robotParts.railParts.fakeMotor.getSpeed());
  }

  /**
   * Construct fake parts for all the subsystems.
   *
   * <p>This is used so we can construct a fake robot and get at all
   * the individual devices within the subsytems to manipulate them in
   * our tests.
   */
  private class FakeParts {
    public final RobotConfig robotConfig = new RobotConfig();
    public final ExampleRailTest.FakeParts railParts
        = new ExampleRailTest.FakeParts();
    public final FakeBuiltinSubsystemParts builtinParts
        = new FakeBuiltinSubsystemParts();

    /**
     * Return a factory populated with devices for all subsystems.
     */
    public MappedDeviceFactory makeFactory() {
      // We only have one subsystem so can just use the test device factory.
      // But if we had multiple subsystems we'd want to aggregate them all
      // together. So we'll aggregate here as if we had multiple to show how.
      MappedDeviceFactory deviceFactory = new MappedDeviceFactory();
      deviceFactory.update(railParts.deviceFactory);
      deviceFactory.update(builtinParts.deviceFactory);
      return deviceFactory;
    }
  }
}
