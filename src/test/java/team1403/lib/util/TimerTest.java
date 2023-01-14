package team1403.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Consumer;
import java.util.function.ToIntBiFunction;
import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import team1403.lib.util.test.FakeClock;


/**
 * Test fixture for BaseTimer.
 */
class TimerTest {

  @Test
  void testElapsedTime() {
    FakeClock clock = new FakeClock();
    clock.advanceSecs(3.21);

    BaseTimer timer = new BaseTimer(
        "TestTimer", clock, BaseTimer.RecordUnit.MILLISECOND);
    assertEquals("TestTimer", timer.getName());

    assertEquals(0, timer.nowMicros());
    assertEquals(0.0, timer.nowMillis());
    
    double expectMillis = 1.23;
    long expectMicros = (long)(1.23 * 1000);
    clock.advanceSecs(expectMicros * Clock.kSecsPerMicro);

    assertEquals(expectMicros, timer.nowMicros());
    assertEquals(expectMillis, timer.nowMillis());
    clock.advanceSecs(500 * Clock.kSecsPerMicro);

    assertEquals(expectMicros + 500, timer.nowMicros());
    assertEquals(expectMillis + 0.5, timer.nowMillis());

    timer.restart();
    assertEquals(0, timer.nowMicros());
    assertEquals(0.0, timer.nowMillis());
    clock.advanceSecs(2);
    assertEquals(2 * Clock.kMicrosPerSec, timer.nowMicros());
    assertEquals(2 * Clock.kMillisPerSec, timer.nowMillis());
  }

  @Test
  void testRecordTime() {
    FakeClock clock = new FakeClock();
    clock.advanceSecs(1234.575);
    BaseTimer microTimer = new BaseTimer(
        "MicroTimer", clock, BaseTimer.RecordUnit.MICROSECOND);
    BaseTimer milliTimer = new BaseTimer(
        "MilliTimer", clock, BaseTimer.RecordUnit.MILLISECOND);

    // Test can ask for micros regardless of recording units
    // and timer will record using units specified at construction.
    clock.advanceSecs(0.5);
    assertEquals(0.5 * Clock.kMicrosPerSec, microTimer.recordMicros());
    assertEquals(0.5 * Clock.kMicrosPerSec, milliTimer.recordMicros());

    try (final NetworkTableEntry microEntry = SmartDashboard.getEntry("MicroTimer");
         final NetworkTableEntry milliEntry = SmartDashboard.getEntry("MilliTimer")) {
      assertEquals(0.5 * Clock.kMicrosPerSec, microEntry.getNumber(-1));
      assertEquals(0.5 * Clock.kMillisPerSec, milliEntry.getDouble(-1));

      // Test can ask for millis regardless of recording units
      // and timer will record using units specified at construction.
      clock.advanceSecs(0.25);
      assertEquals(0.75 * Clock.kMillisPerSec, microTimer.recordMillis());
      assertEquals(0.75 * Clock.kMillisPerSec, milliTimer.recordMillis());

      assertEquals(0.75 * Clock.kMicrosPerSec, microEntry.getNumber(-1));
      assertEquals(0.75 * Clock.kMillisPerSec, milliEntry.getDouble(-1));
    }
  }


  @SuppressWarnings("PMD.ExcessiveMethodLength")
  @Test
  void testTimer() {
    // The clock we'll time with.
    final FakeClock clock = new FakeClock();

    // We're going to start at the given time.
    final double startTime = 1234.575;
    clock.advanceSecs(startTime);

    // Each timeCallback will take 0.25 seconds.
    final double expectSecs = 0.25;
    final Runnable callback = new Runnable() {
        @Override
        public void run() {
          clock.advanceSecs(expectSecs);
        }
    };

    // These are the expected results of timeCallback depending on variant.
    final double expectMillis = expectSecs * Clock.kMillisPerSec;
    final long expectMicros = (long)(expectSecs * Clock.kMicrosPerSec);

    // When we record the timer, this is where it should record to.
    try (final NetworkTableEntry microEntry = SmartDashboard.getEntry("MicroCallbackTimer");
         final NetworkTableEntry milliEntry = SmartDashboard.getEntry("MilliCallbackTimer")) {
      // Helper function for testing whether or not we recorded.
      // Note this assumes that we test unrecorded calls first.
      // Written as a lambda for convienence of capturing local
      // variables rather than passing them through as parameters.
      final Consumer<Boolean> testRecorded = (Boolean record) -> {
        if (record.booleanValue()) {
          // network tables treat all numbers as doubles.
          assertEquals((double)expectMicros, microEntry.getNumber(-1));
          assertEquals(expectMillis, milliEntry.getDouble(-1));
        } else {
          // We didnt record so nothing written yet.
          assertEquals(-1.0, microEntry.getNumber(-1));
          assertEquals(-1.0, milliEntry.getDouble(-1.0));
        }
      };

      // The timer's that we'll test. We construct one for each unit type.
      final BaseTimer microTimer = new BaseTimer(
          "MicroCallbackTimer", clock, BaseTimer.RecordUnit.MICROSECOND);
      final BaseTimer milliTimer = new BaseTimer(
          "MilliCallbackTimer", clock, BaseTimer.RecordUnit.MILLISECOND);

      // Tests timing as milliseconds. The timers should return milliseconds
      // but record as their constructed units (if recording at all).
      // Written as a lambda for convienence of capturing local
      // variables rather than passing them through as parameters.
      final ToIntBiFunction<Integer, Boolean> testMillis
          = (Integer numCallsSoFar, Boolean record) -> {
            int count = numCallsSoFar.intValue();
            assertEquals(startTime + count * expectSecs,
                         clock.nowSecs());
            assertEquals(expectMillis,
                         milliTimer.timeCallMillis(record, callback));
            ++count;
            assertEquals(startTime + count * expectSecs,
                         clock.nowSecs());

            assertEquals(expectMillis,
                         microTimer.timeCallMillis(record, callback));
            ++count;
            assertEquals(startTime + count * expectSecs,
                         clock.nowSecs());
            return count;
          };

      // Similar to testMillis but asks for microseconds.
      final ToIntBiFunction<Integer, Boolean> testMicros
          = (Integer numCallsSoFar, Boolean record) -> {
            int count = numCallsSoFar.intValue();
            assertEquals(startTime + count * expectSecs,
                         clock.nowSecs());
            assertEquals(expectMicros,
                         milliTimer.timeCallMicros(record, callback));
            ++count;
            assertEquals(startTime + count * expectSecs,
                         clock.nowSecs());

            assertEquals(expectMicros,
                         microTimer.timeCallMicros(record, callback));
            ++count;
            assertEquals(startTime + count * expectSecs,
                         clock.nowSecs());
            return count;
          };


      // Run the variations.
      int numCallsSoFar = 0;

      // Test unrecorded timing
      numCallsSoFar = testMillis.applyAsInt(numCallsSoFar, false);
      testRecorded.accept(false);
      numCallsSoFar = testMicros.applyAsInt(numCallsSoFar, false);
      testRecorded.accept(false);

      // Test recorded timing
      numCallsSoFar = testMillis.applyAsInt(numCallsSoFar, true);
      testRecorded.accept(true);
      testMicros.applyAsInt(numCallsSoFar, true);
      testRecorded.accept(true);
    }
  }
}
