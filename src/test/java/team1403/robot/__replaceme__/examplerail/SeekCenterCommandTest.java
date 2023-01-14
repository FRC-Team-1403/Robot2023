package team1403.robot.__replaceme__.examplerail;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.robot.__replaceme__.RobotConfig;


/**
 * Test fixture for SeekCenterCommand.
 */
@SuppressWarnings("PMD.NcssCount")
class SeekCenterCommandTest {
  @Test
  void testSeek() {
    final double kTolerance = 3.0;
    final double kMotorSpeed = 0.175;
    final var robotConfig = new RobotConfig();
    robotConfig.exampleRail.motorSpeed = kMotorSpeed;
    final var parts = new ExampleRailTest.FakeParts();
    final var parameters = new CougarLibInjectedParameters.Builder()
        .deviceFactory(parts.deviceFactory)
        .build();
    final var rail = new ExampleRail(parameters, robotConfig);
    final var motor = parts.fakeMotor;

    final SeekCenterCommand command = new SeekCenterCommand(rail, kTolerance);
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

    assertEquals(Double.NaN, rail.getFrontPositionTicks());
    assertEquals(Double.NaN, rail.getBackPositionTicks());

    parts.fakeEncoder.setPositionTicks(111);
    parts.backLimitSwitch.setTriggered(true);  // has no effect.
    for (int i = 0; i < 3; ++i) {
      scheduler.run();
      assertEquals(stopCalls, motor.countStopMotorCalls());
      assertEquals(-kMotorSpeed, motor.getSpeed());
      assertEquals(Double.NaN, rail.getFrontPositionTicks());
      assertEquals(Double.NaN, rail.getBackPositionTicks());
    }
    parts.backLimitSwitch.setTriggered(false);
    parts.frontLimitSwitch.setTriggered(true);
    scheduler.run();
    assertEquals(stopCalls + 1, motor.countStopMotorCalls());
    assertEquals(0.0, motor.getSpeed());
    assertEquals(111, rail.getFrontPositionTicks());
    assertEquals(Double.NaN, rail.getBackPositionTicks());

    scheduler.run();
    assertEquals(kMotorSpeed, motor.getSpeed());  // moving toward back now
    assertEquals(stopCalls + 1, motor.countStopMotorCalls());
    assertEquals(111, rail.getFrontPositionTicks());
    assertEquals(Double.NaN, rail.getBackPositionTicks());

    parts.frontLimitSwitch.setTriggered(true);  // does not matter
    for (int i = 0; i < 6; i++) {
      scheduler.run();
      assertEquals(kMotorSpeed, motor.getSpeed());  // moving toward back now
      assertEquals(stopCalls + 1, motor.countStopMotorCalls());
      assertEquals(111, rail.getFrontPositionTicks());
      assertEquals(Double.NaN, rail.getBackPositionTicks());
    }

    parts.fakeEncoder.setPositionTicks(333);
    parts.backLimitSwitch.setTriggered(true);
    scheduler.run();
    assertEquals(0.0, motor.getSpeed());
    assertEquals(stopCalls + 2, motor.countStopMotorCalls());
    assertEquals(111, rail.getFrontPositionTicks());
    assertEquals(333, rail.getBackPositionTicks());

    parts.fakeEncoder.setPositionTicks(250);
    scheduler.run();
    assertEquals(-kMotorSpeed, motor.getSpeed());
    assertEquals(stopCalls + 2, motor.countStopMotorCalls());

    // overshot so should move back other way
    parts.fakeEncoder.setPositionTicks(200);
    scheduler.run();
    assertEquals(kMotorSpeed, motor.getSpeed());
    assertEquals(stopCalls + 2, motor.countStopMotorCalls());

    // overshot backward but within tolerance
    parts.fakeEncoder.setPositionTicks(224);
    scheduler.run();
    assertEquals(0.0, motor.getSpeed());
    // we explicitly stop when we hit middle, then again when command ends.
    assertEquals(stopCalls + 3 + 1, motor.countStopMotorCalls());

    assertFalse(command.isScheduled());
  }
}
