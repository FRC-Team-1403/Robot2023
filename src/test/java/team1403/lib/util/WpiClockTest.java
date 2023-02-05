package team1403.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.Instant;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Timer;


/**
 * Test fixture for WpiClock.
 */
class WpiClockTest {
  private static final WpiClock clock = WpiClock.instance();

  @Test
  void testNow() {
    long microseconds = clock.nowMicros();  // NOPMD
    double secs = clock.nowSecs();  // NOPMD
    double milliseconds = clock.nowMillis();  // NOPMD
    long microsecondsAgain = clock.nowMicros(); // NOPMD

    assertTrue(microsecondsAgain > microseconds);
    assertEquals(secs * 1000.0, (double)milliseconds, 1);
    assertEquals(microseconds * 0.001, milliseconds, 50);
  }

  @Test
  void testSystemClock() {
    // Seems consistent with our native system clock.
    assertEquals(clock.epochMicros() * 0.001,
                 (double)Instant.now().toEpochMilli(),
                 1);
  }

  @Test
  void testMatchRemainingPeriod() {
    assertEquals(clock.matchSecondsRemainingInPeriod(),
                 Timer.getMatchTime(),
                 0.1);
  }

  /**
   * JUnit5 assertion that lhs > rhs.
   *
   * @param lhs The value we are testing
   * @param rhs The value we are comparing against should be less.
   */
  void assertGreaterThan(double lhs, double rhs) {
    assertTrue(lhs > rhs, () -> String.format("Expected %f > %f", lhs, rhs));
  }

  @Test
  void testRealTime() {
    double before = clock.nowMillis();
    try {
      Thread.sleep(20);
    } catch (InterruptedException ex) {
      // ignore
    }

    // Ideally the time difference would be > 20 ms.
    // However, Thread.sleep sometimes comes up a bit short.
    // At least on Windows and once on github (by almost 3ms).
    // Here we give more headroom to avoid flakes.
    assertGreaterThan(clock.nowMillis() - before, 15);
  }
}


