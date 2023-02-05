package team1403.lib.util.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;



/**
 * Test fixture for FakeClock.
 */
class FakeClockTest {
  static final double kMillisPerSecond = 1000;
  static final double kMicrosPerMilli = 1000;
  static final double kEpsilon = 0.00000001;  // doubles are close enough.

  @Test
  void testFixedIncrement() {
    final double kStartSecs = 10.0;
    final double kDeltaSecs = 0.25;
    FakeClock clock = new FakeClock(kStartSecs, kDeltaSecs);

    // The first time is the start time we specified.
    assertEquals(kStartSecs, clock.nowSecs());
    for (int i = 1; i < 15; i++) {
      // Each subsequent time increments by kDeltaSecs
      assertEquals(kStartSecs + kDeltaSecs * i, clock.nowSecs());
    }

    double secs = clock.nowSecs();
    double millis = clock.nowMillis();  // should have advanced
    double micros = clock.nowMicros();  // should have advanced again

    assertEquals((secs + kDeltaSecs) * kMillisPerSecond, millis);
    assertEquals((millis + (kMillisPerSecond * kDeltaSecs)) * kMicrosPerMilli,
                 micros);

    secs = clock.nowSecs();
    clock.advanceSecs(123);
    assertEquals(secs + 123 + kDeltaSecs, clock.nowSecs());
  }

  @Test
  void testNoIncrement() {
    final double kStartSecs = 10.0;
    FakeClock clock = new FakeClock(kStartSecs, 0.0);
    for (int i = 1; i < 5; i++) {
      assertEquals(kStartSecs, clock.nowSecs());
      assertEquals(kStartSecs * kMillisPerSecond, clock.nowMillis());
      assertEquals(kStartSecs * kMillisPerSecond * kMicrosPerMilli,
                   clock.nowMicros());
    }

    double expectSecs = kStartSecs;
    for (int i = 1; i < 5; i++) {
      clock.advanceSecs(i * 0.1);
      expectSecs += i * 0.1;
      assertEquals(expectSecs, clock.nowSecs(), kEpsilon);
      assertEquals(expectSecs * kMillisPerSecond, clock.nowMillis(), kEpsilon);

      long expectUs = (long)(expectSecs * kMillisPerSecond * kMicrosPerMilli);
      assertEquals(expectUs, clock.nowMicros());
    }

    clock.advanceSecs(0);
    assertEquals(expectSecs, clock.nowSecs());
  }

  @Test
  void testBadConstruction() {
    assertThrows(IllegalArgumentException.class, () -> new FakeClock(-1, 0));
    assertThrows(IllegalArgumentException.class, () -> new FakeClock(0, -1));
  }

  @Test
  void testSequence() {
    FakeClock clock = new FakeClock(1, 2, 3);

    assertEquals(1, clock.nowSecs());
    assertEquals(2, clock.nowSecs());
    assertEquals(3, clock.nowSecs());
    assertThrows(IllegalStateException.class, () -> clock.nowSecs());
  }

  @Test
  void testBadAdvance() {
    FakeClock clock = new FakeClock(0.0, 0.0);
    assertThrows(IllegalArgumentException.class, () -> clock.advanceSecs(-1));
  }
}
