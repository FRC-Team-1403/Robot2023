package team1403.lib.device.wpi;


import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.PowerDistributionFaults;

import team1403.lib.util.CougarLogger;


/**
 * Test some of the unfortunate complexity in WpiPowerDistribution.
 *
 * <p>These tests are only on the logic, we do not talk to actual hardware.
 */
class WpiPowerDistributionTest {
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
  void testFaultBits() {
    final int allCircuits = WpiPowerDistribution.kCircuitBreakerFaultMask;
    {
      var faults =
          new PowerDistributionFaults(WpiPowerDistribution.kBrownoutFaultMask);
      assertTrue(faults.Brownout);
      assertFalse(faults.CanWarning);
      assertFalse(faults.HardwareFault);

      assertFalse(faults.Channel0BreakerFault);
      assertFalse(faults.Channel23BreakerFault);

      assertEquals(WpiPowerDistribution.kBrownoutFaultMask,
                   WpiPowerDistribution.getFaultBits(faults, allCircuits));
    }

    {
      var faults =
          new PowerDistributionFaults(WpiPowerDistribution.kCanFaultMask);
      assertFalse(faults.Brownout);
      assertTrue(faults.CanWarning);
      assertFalse(faults.HardwareFault);

      assertFalse(faults.Channel0BreakerFault);
      assertFalse(faults.Channel23BreakerFault);

      assertEquals(WpiPowerDistribution.kCanFaultMask,
                   WpiPowerDistribution.getFaultBits(faults, allCircuits));
    }

    {
      var faults =
          new PowerDistributionFaults(WpiPowerDistribution.kHardwareFaultMask);
      assertFalse(faults.Brownout);
      assertFalse(faults.CanWarning);
      assertTrue(faults.HardwareFault);

      assertFalse(faults.Channel0BreakerFault);
      assertFalse(faults.Channel23BreakerFault);

      assertEquals(WpiPowerDistribution.kHardwareFaultMask,
                   WpiPowerDistribution.getFaultBits(faults, allCircuits));
    }
  }

  @Test
  void testCircuitBreakerFaultBits() {
    final int allCircuits = WpiPowerDistribution.kCircuitBreakerFaultMask;
    {
      var faults =
          new PowerDistributionFaults(WpiPowerDistribution.kCircuitBreakerFaultMask);
      assertFalse(faults.Brownout);
      assertFalse(faults.CanWarning);
      assertFalse(faults.HardwareFault);

      assertTrue(faults.Channel0BreakerFault);
      assertTrue(faults.Channel23BreakerFault);

      assertEquals(WpiPowerDistribution.kCircuitBreakerFaultMask,
                   WpiPowerDistribution.getFaultBits(faults, allCircuits));
    }

    {
      var faults = new PowerDistributionFaults(0x1 << 10);
      assertFalse(faults.Brownout);
      assertFalse(faults.CanWarning);
      assertFalse(faults.HardwareFault);

      assertFalse(faults.Channel0BreakerFault);
      assertFalse(faults.Channel23BreakerFault);

      assertTrue(faults.Channel10BreakerFault);
      assertEquals(0x1 << 10,
                   WpiPowerDistribution.getFaultBits(faults, allCircuits));
    }

    {
      int bits =
          WpiPowerDistribution.kBrownoutFaultMask
          | WpiPowerDistribution.kCanFaultMask
          | WpiPowerDistribution.kHardwareFaultMask
          | 0x1 << 0
          | 0x1 << 23;

      var faults = new PowerDistributionFaults(bits);
      assertTrue(faults.Brownout);
      assertTrue(faults.CanWarning);
      assertTrue(faults.HardwareFault);
      assertTrue(faults.Channel0BreakerFault);
      assertTrue(faults.Channel23BreakerFault);

      assertFalse(faults.Channel1BreakerFault);
      assertFalse(faults.Channel22BreakerFault);

      int faultBits =  WpiPowerDistribution.getFaultBits(faults, allCircuits);
      assertEquals(bits, faultBits);

      int breakerBits =
          WpiPowerDistribution.breakerFaultBits(faults, allCircuits);

      assertEquals(0x1 << 0 | 0x1 << 23, breakerBits);
    }

    {
      // First 3 circuits = 0x1 | 0x2 | 0x4
      var faults = new PowerDistributionFaults(0x7);
      assertEquals(0x7, WpiPowerDistribution.getFaultBits(faults, 0x7));

      // Interested in circuit 10 only, which is not faulted.
      assertEquals(0x0, WpiPowerDistribution.getFaultBits(faults, 0x1 << 10));

      // We are interested in everything but channel 1  (0x2)
      // So bits should be 0x7 without 0x2, which is 0x5.
      int not1 = WpiPowerDistribution.kCircuitBreakerFaultMask ^ (0x1 << 1);
      assertEquals(0x5, WpiPowerDistribution.getFaultBits(faults, not1));
    }
  }

  @Test
  void testCircuitFaults() {
    var logger = CougarLogger.getCougarLogger("MyLogger");

    // This try is because the PowerDistribution is close
    try (var distributor = new WpiPowerDistribution("MyPDP", logger)) {
      assertEquals("MyPDP", distributor.getName());


      distributor.makeCurrentSensor("TestCircuit0", 0);
      distributor.makeCurrentSensor("TestCircuit8", 8);

      var faults = new PowerDistributionFaults(0x1 << 7 | 0x1 << 8);
      assertEquals("TestCircuit8", distributor.toFaultString(faults));

      faults = new PowerDistributionFaults(0x1 << 0 | 0x1 << 8);
      assertEquals("TestCircuit0 TestCircuit8", distributor.toFaultString(faults));
    
      faults = new PowerDistributionFaults(0x1 << 0 | WpiPowerDistribution.kBrownoutFaultMask);
      assertEquals("Brownout TestCircuit0", distributor.toFaultString(faults));
    }
  }
}
