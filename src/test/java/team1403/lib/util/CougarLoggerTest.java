package team1403.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.logging.Level;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Test fixture for CougarLogger.
 */
@SuppressWarnings("PMD.AvoidDuplicateLiterals")
class CougarLoggerTest {
  @TempDir
  File tempDir;

  private NetworkTable m_baseNetworkTable;

  @BeforeEach
  void prepareNetworkTables() {
    try (NetworkTableInstance instance = NetworkTableInstance.getDefault()) {
      m_baseNetworkTable = instance.getTable(CougarLogger.kTable);
      // We'd like to clear all the keys but there is no longer an API to do that.
    }
  }

  @Test
  @SuppressWarnings("VariableDeclarationUsageDistance")
  void testGetLogger() {
    final String fullClassName = CougarLoggerTest.class.getName();
    final String packageName = CougarLoggerTest.class.getPackage().getName();

    final CougarLogger byName = CougarLogger.getCougarLogger(packageName);
    assertEquals(packageName, byName.getName());
    assertTrue(m_baseNetworkTable.containsKey(packageName));

    assertFalse(m_baseNetworkTable.containsKey(fullClassName));
    final String shortClassName = fullClassName.substring(packageName.length() + 1);
    final CougarLogger byChild = CougarLogger.getChildLogger(byName, shortClassName);
    assertEquals(fullClassName, byChild.getName());
    assertTrue(m_baseNetworkTable.containsKey(fullClassName));

    final CougarLogger byClass = CougarLogger.getLoggerForClass(CougarLoggerTest.class);
    assertSame(byChild, byClass);
  }

  @Test
  void testIsConsoleLoggable() {
    final CougarLogger logger = CougarLogger.getCougarLogger("testConsoleLogger");
    final CougarLogger child = CougarLogger.getChildLogger(logger, "consoleChild");

    // The child's level is not going to affect console logging.
    child.setLevel(Level.FINEST);

    logger.setConsoleLevel(Level.FINE);
    assertTrue(child.isLoggable(Level.FINEST));
    assertFalse(logger.isConsoleLoggable(Level.FINEST));
    assertFalse(child.isConsoleLoggable(Level.FINEST));
    assertTrue(logger.isConsoleLoggable(Level.FINE));
    assertTrue(child.isConsoleLoggable(Level.FINE));

    child.setConsoleLevel(Level.FINEST);
    assertFalse(logger.isConsoleLoggable(Level.FINEST));
    assertTrue(child.isConsoleLoggable(Level.FINEST));

    child.setConsoleLevel(Level.INFO);
    assertTrue(child.isLoggable(Level.FINEST));
    assertFalse(logger.isConsoleLoggable(Level.FINEST));
    assertFalse(child.isConsoleLoggable(Level.FINEST));
    assertTrue(logger.isConsoleLoggable(Level.FINE));
    assertFalse(child.isConsoleLoggable(Level.FINE));
    assertTrue(logger.isConsoleLoggable(Level.INFO));
    assertTrue(child.isConsoleLoggable(Level.INFO));

    child.setConsoleLevel(null);
    assertTrue(child.isLoggable(Level.FINEST));
    assertFalse(child.isConsoleLoggable(Level.FINEST));
    assertTrue(child.isConsoleLoggable(Level.FINE));
  }

  @Test
  void cannotChangeAlwaysOn() {
    CougarLogger alwaysOn = CougarLogger.getAlwaysOn();
    alwaysOn.setLevel(Level.INFO);
    assertEquals(Level.ALL, alwaysOn.getLevel());
    alwaysOn.setConsoleLevel(Level.INFO);
    assertEquals(Level.ALL, alwaysOn.getConsoleLevel());
  }

  @Test
  void testIsLoggable() {
    final CougarLogger logger = CougarLogger.getCougarLogger("testLogger");
    final CougarLogger child = CougarLogger.getChildLogger(logger, "Child");

    logger.setLevel(Level.FINE);
    assertFalse(logger.isLoggable(Level.FINEST));
    assertFalse(child.isLoggable(Level.FINEST));
    assertTrue(logger.isLoggable(Level.FINE));
    assertTrue(child.isLoggable(Level.FINE));
    assertTrue(logger.isLoggable(Level.INFO));
    assertTrue(child.isLoggable(Level.INFO));

    logger.setLevel(Level.INFO);
    assertFalse(logger.isLoggable(Level.FINEST));
    assertFalse(child.isLoggable(Level.FINEST));
    assertFalse(logger.isLoggable(Level.FINE));
    assertFalse(child.isLoggable(Level.FINE));
    assertTrue(logger.isLoggable(Level.INFO));
    assertTrue(child.isLoggable(Level.INFO));

    logger.setLevel(Level.FINEST);
    assertTrue(logger.isLoggable(Level.FINEST));
    assertTrue(child.isLoggable(Level.FINEST));
    assertTrue(logger.isLoggable(Level.FINE));
    assertTrue(child.isLoggable(Level.FINE));
    assertTrue(logger.isLoggable(Level.INFO));
    assertTrue(child.isLoggable(Level.INFO));

    child.setLevel(Level.FINE);
    assertFalse(child.isLoggable(Level.FINEST));
    assertTrue(child.isLoggable(Level.FINE));
    assertTrue(child.isLoggable(Level.INFO));

    logger.setLevel(Level.FINEST);
    assertTrue(logger.isLoggable(Level.FINEST));
    assertFalse(child.isLoggable(Level.FINEST));
    assertTrue(child.isLoggable(Level.FINE));

    child.setLevel(null);
    assertTrue(child.isLoggable(Level.FINEST));

    logger.setLevel(Level.INFO);
    assertFalse(child.isLoggable(Level.FINEST));
  }

  /**
   * Create a property file to initialize Java Logging.
   *
   * <p>This is a helper function used to test initialization from a
   * property file. See the config/test.logging.properties for more
   * information. We are not using that file here because this is
   * testing other configuration options and we want control over
   * that in the test.
   *
   * <p>The generated property file will be written to the given path.
   *
   * @param propertyPath The path to the property file written, or null
   * @param baseLevel The default Logging level to write in the config.
   * @param testLevel the logging level to specify for "team1403.lib.MyTest".
   *
   * @return propertyPath on success or null on failure.
   */
  @SuppressWarnings("PMD.CloseResource")
  private Path createPropertyFile(Path propertyPath, Level baseLevel, Level testLevel) {
    try {
      File logFile = new File(tempDir, "testlog");
      PrintStream stream = new PrintStream(Files.newOutputStream(propertyPath));
      stream.println("handlers=java.util.logging.FileHandler");
      String logPathStr = logFile.getPath().replace("\\", "/");
      stream.println("java.util.logging.FileHandler.pattern=" + logPathStr);
      stream.println(
          "java.util.logging.FileHandler.formatter = java.util.logging.SimpleFormatter");
      stream.println("java.util.logging.SimpleFormatter.format = [%4$s] %5$s%n");
      stream.println("java.util.logging.FileHandler.level = FINEST");
      // Set the base level
      stream.println(".level = " + baseLevel.getName());
      // Override the level for our "MyTest" logger
      stream.println("team1403.lib.MyTest.level = " + testLevel.getName());
      stream.close();
      return Paths.get(logFile.getPath());
    } catch (IOException ioex) {
      return null;
    }
  }

  /**
   * Tests the module initializer from a properties file.
   */
  @Test
  @SuppressWarnings({"PMD.CognitiveComplexity",
                     "PMD.NcssCount",
                     "PMD.ExcessiveMethodLength",
                     "PMD.AvoidInstantiatingObjectsInLoops"})
  void testInitFromPropertiesFile() {
    CougarLogger.resetForTesting();

    // Test two ways (and we can reinitialize logging)
    //    First way default is INFO but we override a particular class to FINE
    //    Second way default is FINE but we override a particular class to INFO
    for (int i = 0; i < 3; ++i) {
      Level baseLevel = i == 0
                      ? Level.INFO
                      : (i == 1 ? Level.FINE : Level.FINEST);
      Level testLevel = i == 0
                      ? Level.FINEST
                      : (i == 1 ? Level.FINE : Level.INFO);
      File propertyFile = new File(tempDir, "config.properties");
      Path propertyPath = Paths.get(propertyFile.getPath());
      Path propertyLogPath = createPropertyFile(propertyPath, baseLevel, testLevel);
      Path fullLogPath = new File(tempDir, "AutoDebugLog.log").toPath();
      assertNotNull(propertyPath);
      assertTrue(CougarLogger.initModule(propertyPath, fullLogPath));

      // Logs at "MyTest" level
      CougarLogger logger = CougarLogger.getCougarLogger("team1403.lib.MyTest");
      assertEquals(testLevel, logger.getLevel());
      logger.tracef("My Trace");
      logger.debugf("My Debug");
      logger.infof("My Info");

      // Child logger
      CougarLogger childLogger = CougarLogger.getChildLogger(logger, "Child");
      // java Logger uses null levels to denote inheritence
      assertEquals(null, childLogger.getLevel());
      childLogger.tracef("Child Trace");
      childLogger.debugf("Child Debug");
      childLogger.infof("Child Info");

      // Logs in "subpackage"
      // Java doesnt have subpackages, so these will be at the base level
      CougarLogger subpackageLogger = CougarLogger.getCougarLogger("team1403.lib.sub.SubPackage");
      assertEquals(null, subpackageLogger.getLevel());
      subpackageLogger.tracef("Subpackage Trace");
      subpackageLogger.debugf("Subpackage Debug");
      subpackageLogger.infof("Subpackage Info");

      // Logs at other (inherited from base)
      CougarLogger otherLogger = CougarLogger.getCougarLogger("team1403.robots");
      assertEquals(null, otherLogger.getLevel());

      // This is the same, but we are testing the format string also
      otherLogger.tracef("%s Trace", "Other");
      otherLogger.debugf("%s Debug", "Other");
      otherLogger.infof("%s Info", "Other");

      // We'll just test warningf and errorf on this logger.
      otherLogger.warningf("%s %s %s", "Test", "warning", "message");
      otherLogger.errorf("%s code=%d value=%.2f", "Error", 123, 3.14);

      // Check base level (what other inherited from) is what we set.
      assertEquals(baseLevel, CougarLogger.getCougarLogger("").getLevel());

      // Always logs regardless of level settings.
      // But handler will still determine if it will be rendered.
      CougarLogger.getAlwaysOn().tracef("AlwaysOn");

      final String eoln = System.lineSeparator();
      String expectProperty;
      String expectFull;
 
      // Note that CougarLogger replaces the file handler's formatter
      // with an explicit one.
      if (testLevel.equals(Level.FINEST)) {
        expectProperty = String.join(eoln,
                             "[FINEST] My Trace",
                             "[FINE] My Debug",
                             "[INFO] My Info",
                             "[FINEST] Child Trace",
                             "[FINE] Child Debug",
                             "[INFO] Child Info",

                             "[INFO] Subpackage Info",
                             "[INFO] Other Info",
                             "[WARNING] Test warning message",
                             "[SEVERE] Error code=123 value=3.14",

                             "[FINEST] AlwaysOn",
                             ""); // end with line-separator
        expectFull = String.join(eoln,
                             "T [team1403.lib.MyTest] My Trace",
                             "D [team1403.lib.MyTest] My Debug",
                             "I [team1403.lib.MyTest] My Info",
                             "T [team1403.lib.MyTest.Child] Child Trace",
                             "D [team1403.lib.MyTest.Child] Child Debug",
                             "I [team1403.lib.MyTest.Child] Child Info",

                             "I [team1403.lib.sub.SubPackage] Subpackage Info",
                             "I [team1403.robots] Other Info",
                             "W [team1403.robots] Test warning message",
                             "E [team1403.robots] Error code=123 value=3.14",

                             "T [team1403.logger.AlwaysOn] AlwaysOn",
                             ""); // end with line-separator
      } else if (testLevel.equals(Level.FINE)) {
        expectProperty = String.join(eoln,
                             "[FINE] My Debug",
                             "[INFO] My Info",
                             "[FINE] Child Debug",
                             "[INFO] Child Info",

                             "[FINE] Subpackage Debug",
                             "[INFO] Subpackage Info",
                             "[FINE] Other Debug",
                             "[INFO] Other Info",
                             "[WARNING] Test warning message",
                             "[SEVERE] Error code=123 value=3.14",

                             "[FINEST] AlwaysOn",
                             ""); // end with line-separator
        expectFull = String.join(eoln,
                             "D [team1403.lib.MyTest] My Debug",
                             "I [team1403.lib.MyTest] My Info",
                             "D [team1403.lib.MyTest.Child] Child Debug",
                             "I [team1403.lib.MyTest.Child] Child Info",

                             "D [team1403.lib.sub.SubPackage] Subpackage Debug",
                             "I [team1403.lib.sub.SubPackage] Subpackage Info",
                             "D [team1403.robots] Other Debug",
                             "I [team1403.robots] Other Info",
                             "W [team1403.robots] Test warning message",
                             "E [team1403.robots] Error code=123 value=3.14",

                             "T [team1403.logger.AlwaysOn] AlwaysOn",
                             ""); // end with line-separator
      } else {
        expectProperty = String.join(eoln,
                             "[INFO] My Info",
                             "[INFO] Child Info",

                             "[FINEST] Subpackage Trace",
                             "[FINE] Subpackage Debug",
                             "[INFO] Subpackage Info",
                             "[FINEST] Other Trace",
                             "[FINE] Other Debug",
                             "[INFO] Other Info",
                             "[WARNING] Test warning message",
                             "[SEVERE] Error code=123 value=3.14",

                             "[FINEST] AlwaysOn",
                             ""); // end with line-separator
        expectFull = String.join(eoln,
                             "I [team1403.lib.MyTest] My Info",
                             "I [team1403.lib.MyTest.Child] Child Info",

                             "T [team1403.lib.sub.SubPackage] Subpackage Trace",
                             "D [team1403.lib.sub.SubPackage] Subpackage Debug",
                             "I [team1403.lib.sub.SubPackage] Subpackage Info",
                             "T [team1403.robots] Other Trace",
                             "D [team1403.robots] Other Debug",
                             "I [team1403.robots] Other Info",
                             "W [team1403.robots] Test warning message",
                             "E [team1403.robots] Error code=123 value=3.14",

                             "T [team1403.logger.AlwaysOn] AlwaysOn",
                             ""); // end with line-separator
      }

      try {
        System.out.println(String.format("baseLevel=%s testLevel=%s",   // NOPMD
                                         baseLevel, testLevel));
        String got = new String(Files.readAllBytes(propertyLogPath));
        assertEquals(expectProperty, got);
        String gotAll = new String(Files.readAllBytes(fullLogPath));

        java.util.function.Function<java.util.regex.MatchResult, String> replacer
            = (java.util.regex.MatchResult result) -> {
              return result.group(1);
            };


        // Remove time from first line but leave the leading level.
        // The time ends with the '[' marking the logger.
        // (identified with \n ending previous entry).
        String cleaned = java.util.regex.Pattern.compile("^(. )[^\\[]*")
            .matcher(gotAll)
            .replaceFirst(replacer);

        // Remove the leading time from each line but the first
        // and remove the time as above.
        cleaned = java.util.regex.Pattern.compile("(\n. )[^\\[]*")
            .matcher(cleaned)
            .replaceAll(replacer);

        assertEquals(expectFull, cleaned);
      } catch (IOException ioex) {
        fail(ioex.toString());
      }
      CougarLogger.resetForTesting();
    }
  }

  /**
   * Test initializing the module with a bad property file.
   *
   * <p>If the file is bad, we should ignore it.
   */
  @Test
  void testInitModuleFailure() {
    // Emit a warning message because the failure is going to emit an error
    // which can be confusing. Printing these types of messages in a test
    // is not normal, but this is so low level that we dont have the logger
    // to write the message so it is less clear that it came from us.
    System.err.println("Testing load failure..."); // NOPMD
    Path fullLogPath = new File(tempDir, "FailureDebugLog.log").toPath();
    assertFalse(
        CougarLogger.initModule(Paths.get("unknownPath"), fullLogPath));
    System.err.println("Saw expected failure..."); // NOPMD
    CougarLogger.resetForTesting();
  }

  @Test
  void testUpdatesNetworkTable() {
    final String kParentKey = "ToParent";
    final String kChildKey = "ToParent.Child";
    assertFalse(m_baseNetworkTable.containsKey(kParentKey));
    final CougarLogger parentLogger = CougarLogger.getCougarLogger(kParentKey);
    assertTrue(m_baseNetworkTable.containsKey(kParentKey));

    assertFalse(m_baseNetworkTable.containsKey(kChildKey));
    final CougarLogger childLogger = CougarLogger.getChildLogger(parentLogger, "Child");
    assertTrue(m_baseNetworkTable.containsKey(kChildKey));

    try (final NetworkTableEntry parentEntry = m_baseNetworkTable.getEntry(kParentKey);
         final NetworkTableEntry childEntry = m_baseNetworkTable.getEntry(kChildKey)) {
      assertEquals(0, parentEntry.getInteger(-1));
      assertEquals(0, childEntry.getInteger(-1));

      parentLogger.setConsoleLevel(Level.FINEST);
      assertEquals(2, parentEntry.getInteger(-1));
      assertEquals(0, childEntry.getInteger(-1));

      parentLogger.setConsoleLevel(Level.FINE);
      assertEquals(1, parentEntry.getInteger(-1));
      assertEquals(0, childEntry.getInteger(-1));

      parentLogger.setConsoleLevel(Level.INFO);
      assertEquals(0, parentEntry.getInteger(-1));
      assertEquals(0, childEntry.getInteger(-1));

      parentLogger.setConsoleLevel(Level.WARNING);
      assertEquals(0, parentEntry.getInteger(-1));
      assertEquals(0, childEntry.getInteger(-1));

      childLogger.setConsoleLevel(Level.FINE);
      assertEquals(0, parentEntry.getInteger(-1));
      assertEquals(1, childEntry.getInteger(-1));
    }
  }
}
