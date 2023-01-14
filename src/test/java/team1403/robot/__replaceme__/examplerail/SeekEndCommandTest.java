package team1403.robot.__replaceme__.examplerail;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.virtual.ManualLimitSwitch;
import team1403.robot.__replaceme__.RobotConfig;


/**
 * Test fixture for SeekEndCommand.
 */
class SeekEndCommandTest {
  @SuppressWarnings("PMD.NcssCount")
  void doTest(SeekEndCommand.Position position, boolean virtualBackSwitch) {
    final long kRailLengthIfVirtual = 1234;
    final long kRailFrontIfVirtual = kRailLengthIfVirtual - 10;
    final double kMotorSpeed = 0.175;
    final var robotConfig = new RobotConfig();
    robotConfig.exampleRail.motorSpeed = kMotorSpeed;
    if (virtualBackSwitch) {
      robotConfig.ports.exampleRailReverseLimitSwitch = -1;
      robotConfig.exampleRail.virtualBackLimitSwitchTicks = kRailLengthIfVirtual;
    }
    final var parts = new ExampleRailTest.FakeParts();
    final var parameters = new CougarLibInjectedParameters.Builder()
      .deviceFactory(parts.deviceFactory)
      .build();
    final var rail = new ExampleRail(parameters, robotConfig);
    final var motor = parts.fakeMotor;

    final SeekEndCommand command = new SeekEndCommand(rail, position);
    int stopCalls = motor.countStopMotorCalls();

    final var scheduler = CommandScheduler.getInstance();

    // The command has no effect while the robot is disabled.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    assertTrue(DriverStationSim.getEnabled());
    command.schedule();
    assertTrue(command.isScheduled());
    // The command does not alter the current motor speed until it is run.
    assertEquals(Double.NaN, motor.getSpeed());


    ManualLimitSwitch testSwitch;
    ManualLimitSwitch otherSwitch;
    double expectSpeed;
    String expectName;
    if (position == SeekEndCommand.Position.FRONT) {
      expectSpeed = -kMotorSpeed;
      testSwitch = parts.frontLimitSwitch;
      otherSwitch = parts.backLimitSwitch;
      expectName = "SeekRailFRONT";
    } else {
      assertEquals(position, SeekEndCommand.Position.BACK);
      expectSpeed = kMotorSpeed;
      testSwitch = parts.backLimitSwitch;
      otherSwitch = parts.frontLimitSwitch;
      expectName = "SeekRailBACK";
      if (virtualBackSwitch) {
        rail.setFrontPositionTicks(kRailFrontIfVirtual);
        parts.fakeEncoder.setPositionTicks(
            kRailFrontIfVirtual + kRailLengthIfVirtual - 1);
      }
    }

    assertEquals(expectName, command.getName());

    scheduler.run();
    assertEquals(expectSpeed, motor.getSpeed());
    scheduler.run();
    assertEquals(expectSpeed, motor.getSpeed());

    // Trigger the other limit switch, it should be ignored.
    otherSwitch.setTriggered(true);
    scheduler.run();
    assertEquals(expectSpeed, motor.getSpeed());
    assertTrue(command.isScheduled());

    // And again to make sure we arent off by one in checking switch.
    scheduler.run();
    assertEquals(expectSpeed, motor.getSpeed());
    assertTrue(command.isScheduled());

    // Now trigger the switch we are moving towards.
    // It should terminate the command and stop the motor.
    otherSwitch.setTriggered(false);
    if (virtualBackSwitch && position == SeekEndCommand.Position.BACK) {
      parts.fakeEncoder.setPositionTicks(
          kRailFrontIfVirtual + kRailLengthIfVirtual);
    } else {
      testSwitch.setTriggered(true);
    }
    assertEquals(stopCalls, motor.countStopMotorCalls());
    scheduler.run();
    assertEquals(stopCalls + 1, motor.countStopMotorCalls());
    assertEquals(0, motor.getSpeed());
    assertFalse(command.isScheduled());
    assertEquals(stopCalls + 1, motor.countStopMotorCalls());
  }

  @Test
  @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")  // doTest asserts
  void testForward() {
    doTest(SeekEndCommand.Position.FRONT, true);
  }

  @Test
  @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")  // doTest asserts
  void testReverse() {
    doTest(SeekEndCommand.Position.BACK, true);
  }

  @Test
  @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")  // doTest asserts
  void testReverseWithVirtualLimitSwitch() {
    doTest(SeekEndCommand.Position.BACK, false);
  }
}
