package team1403.lib.util;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.ConsoleHandler;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A wrapper around {@code java.util.logging.Logger} for logging.
 *
 * <p>This API is similar to java.util.logging.Logger but is simplified,
 * supports formatted strings and integrated with NetworkTables so the
 * logging levels can by dynamically changed from the driver station to
 * facilitate on-the-spot troubleshooting.
 *
 * <p>Internally it is built on top of java.util.logging.Logger so can
 * be used the same way as far as handlers and configuration.
 *
 * <p><b>Usage</b>:
 * Loggers are created using a variant CougarLogger.getLogger*, similar to
 * the standard java Logger class. However consider composing subsystem loggers
 * so, for example if the robot has logger `l`, then subsystems should
 * have logger CougarLogger.getChildLogger(l, subsystemName).
 * This way one can turn logging on for the whole robot or for subsets of
 * subsystems.
 *
 * <p>Other usage overview:
 * <ul>
 *   <li>Call {@link #initModule initModule} to initialize the logging package.
 *   <li>Properties are configured in the {@code deploy/logging.properties} file.
 *       <ul>
 *       <li> If the logger name is scoped to a specific capability then
 *            the level can be controlled when diagnosing or debugging to
 *            get more information without being deluged.
 *       <li> This could be helpful since logging to the console will have
 *            significant performance impact due to the networking with
 *            the Driver Station.
 *       </ul>
 *    <li> {@link #tracef tracef} for very detailed information.
 *    <li> {@link #debugf debugf} for minimal information useful to debug.
 *    <li> {@link #infof infof} should be rare because of the performance cost.
 *    <li> {@link #warningf warningf}, {@link #errorf errorf} to grab operator
 *         attention. these should be rare as well.
 *    <li> The following levels are used when updating from a Network Table.
 *         Note this is just 2 levels for practicality and simplicity for UI
 *         given the debugging use-case this is for.
 *         <table>
 *             <caption>Supported Log Levels</caption>
 *             <tr><th>Value<th>Level
 *             <tr><td>&ge;2<td>FINEST
 *             <tr><td>1<td>FINE
 *             <tr><td>&le;0<th>null (inherit)
 *         </table>
 *  </ul>
 *
 * <p>For a basic understanding of the Java Logging framework, see
 * [this article](https://www.loggly.com/ultimate-guide/java-logging-basics/)
 * or search for other explanations or tutorials.
 *
 * <p>This is a bit different from the Java Logging framework:
 *     * We force a file logger that logs everything above a certain log level.
 *     * The console logger uses a filter to selectively log at per-logger
 *       levels.
 *     * The console log levels are controlled through network tables so they
 *       can be turned on and off dynamically while the robot is running.
 *
 * <p>This also supports a properties file for historical reasons.
 * The properties file support needs to be re-thought. The point of keeping
 * it would be to control individual modules to be other than "ALL" level.
 * This would be for performance considerations where rendering the log strings
 * for the file logger (and perhaps emitting them) is simply too much overhead.
 *
 * <p>An example file is in {@code config/test.logging.properties}, and
 * is meant for testing. It will write everything a logger emits to the 
 * console.
 *
 * @see java.util.logging.Logger
 */
@SuppressWarnings({"PMD.GodClass", "PMD.TooManyMethods"})
public final class CougarLogger {
  /**
   * The name of a logger that is always enabled and cannot be disabled.
   *
   * <p>It will always write at all levels.
   */
  public static final String kAlwaysOnName = "team1403.logger.AlwaysOn";

  /**
   * The key of the NetworkTable containing the cougar logger levels.
   */
  public static final String kTable = "CougarLoggers";

  /**
   * Maps logging.Level to UI codes for changing the level.
   * We'll use simple numbers since we dont have a means to
   * add an enumeration to the UI that changes these.
   *     0: inherit from parent
   *     1: debug
   *     2: trace
   * If we had an enum in the UI we could show more levels
   * (e.g. to suppress warnings).
   * Since we dont we are keeping it simple to use.
   */
  private static final Map<Level, Integer> levelMap = Map.of(
      Level.FINEST, 2,
      Level.FINE, 1);

  /**
   * Adds logger to NetworkTable and manage changes to its log leve.
   *
   * @param logger The logger to register
   */
  private void register() {
    if (kAlwaysOnName.equals(getName())) {
      // Special case -- this is immutable.
      return;
    }

    var networkTableInstance = NetworkTableInstance.getDefault();
    var table = networkTableInstance.getTable(kTable);
    var tableEntry = table.getEntry(getName());
    var listenEvents = EnumSet.of(NetworkTableEvent.Kind.kValueRemote);

    // Don't keep the resulting handle id since we're never going to stop listening.
    networkTableInstance.addListener(
        tableEntry, listenEvents,
        (NetworkTableEvent event) -> {
          var internalLogger = CougarLogger.getLoggerForClass(CougarLogger.class);
          int num = (int)event.valueData.value.getInteger();
          if (num > 2) { // NOPMD - AvoidLiteralsInIfCondition
            num = 2;
          }

          switch (num) {
            case 2:
              setConsoleLevel(Level.FINEST);
              break;
            case 1:
              setConsoleLevel(Level.FINE);
              break;
            default:
              setConsoleLevel(null);
              break;
          }
          internalLogger.debugf("Updated %s to console level %s",
                                getName(), getConsoleLevel());
        });

    // Pre-populate logger keys to root so can manipulate at any level.
    var name = getName();
    while (! name.isEmpty()) {
      var entry = table.getEntry(getName());
      if (entry.exists()) {
        break;
      }
      entry.setInteger(0);
      var dot = name.lastIndexOf('.');
      if (dot < 0) {
        break;
      }
      name = name.substring(0, dot);
    }
  }


  /**
   * Returns a temporary logger.
   *
   * <p>This logger is intended to be used on a temporary basis
   * to inject some short term logging code. More permanent
   * logging code should be under a specific logger so it can
   * be controlled.
   *
   * <p>The temporary logger is configured with a cranked up logging
   * level so it will emit by default rather than having to
   * temporary configure a new logger with a logging level. This
   * is rather than using System.err.println so you there are
   * potentially additional benefits of going through a Logger.
   *
   * <p>However once the emergency has passed, the use of this
   * logger should be removed or transitioned to a standard
   * logger that can be controlled.
   *
   * @return Logger named {@value #kAlwaysOnName}
   */
  public static CougarLogger getAlwaysOn() {
    return CougarLogger.getCougarLogger(kAlwaysOnName);
  }

  /**
   * Lookup the shared logger based on a given class.
   *
   * @param klass The logger will be for the kclass.getName()
   * @return existing CougarLogger otherwise new one.
   *
   * @see getCougarLogger
   */
  @SuppressWarnings("rawtypes")
  public static CougarLogger getLoggerForClass(Class klass) {
    return getCougarLogger(klass.getName());
  }

  /**
   * Lookup the shared logger based on an existing logger parent.
   *
   * @param parent The logger that the new logger should inherit from
   * @param childName The suffix for the new logger name. The prefix
   *                  will be the parent's name.
   * @return existing CougarLogger otherwise new one.
   *
   * @see getCougarLogger
   */
  public static CougarLogger getChildLogger(CougarLogger parent, String childName) {
    return getCougarLogger(parent.m_logger.getName() + "." + childName);
  }

  /**
   * Returns a logger for the given name.
   *
   * <p>This will create a new logger if one does not exist.
   * This uses the builtin LogManager so is vulnerable to
   * name conflicts with other loggers. If that is the
   * case then this will throw an exception attempting to
   * treat it as a CougarLogger. In practice that should
   * not be a concern.
   *
   * @param name The standard logger name.
   * @return existing CougarLogger otherwise new one.
   */
  public static CougarLogger getCougarLogger(String name) {
    synchronized (_loggers) {
      CougarLogger logger = _loggers.get(name);
      if (logger == null) {
        logger = new CougarLogger(name);
        _loggers.put(name, logger);
        logger.register();
      }
      return logger;
    }
  }

  /**
   * Initialize the logging system from a properties file.
   *
   * @param propertiesPath path to java.logging.properties file.
   * @param debugLogPath path to write debug logs into.
   *
   * @return true if we could initalize off the propertiesPath.
   *         A false result is still valid, but is not using properties.
   */
  public static boolean initModule(Path propertiesPath,
                                   Path debugLogPath) {
    boolean loadedConfig = false;

    // Close the file after processing it.
    try (InputStream configFile = Files.newInputStream(propertiesPath)) {
      LogManager.getLogManager().readConfiguration(configFile);
      loadedConfig = true;
    } catch (IOException ioex) {
      // We're in the logger, so need stderr.
      System.err.println("Could not initialize logging: " + ioex);  // NOPMD
    }

    // Force logger to be created and registered
    // so that player station can change default levels.
    CougarLogger.getCougarLogger("team1403");
    if (!loadedConfig) {
      CougarLogger.getCougarLogger("team1403").setLevel(Level.ALL);
    }

    initConsoleHandler();
    try {
      initFileHandler(debugLogPath);
    } catch (IOException ioex) {
      System.err.println("Could not initialize logging FileHandler: " + ioex); // NOPMD
    }

    return loadedConfig;
  }

  /**
   * Constructor.
   *
   * @param name The name of the logger will be unique to this logger.
   *        The name defines a hierarchy separated by '.'. Prefix
   *        substring form the parents for purposes of
   *        {@link #isLogging isLogging}.
   */
  private CougarLogger(String name) {
    Logger logger = LogManager.getLogManager().getLogger(name);
    if (logger == null) {
      logger = Logger.getLogger(name);
    }
    m_logger = logger;

    if (kAlwaysOnName.equals(name)) {
      m_logger.setLevel(Level.ALL);
      m_consoleLevel = Level.ALL;
    }

    int dot = name.lastIndexOf('.');
    if (dot < 0) {
      m_filterParent = null;
    } else {
      String parentName = name.substring(0, dot);
      m_filterParent = CougarLogger.getCougarLogger(parentName);
    }
  }

  /**
   * Determine if logger logs at the given level.
   *
   * @param level The level to request.
   * @return true if level is at least as high as the requested level.
   */
  public final boolean isLoggable(Level level) {
    return m_logger.isLoggable(level);
  }

  /**
   * Determine if console logs this logger at the given level.
   *
   * @param level The level to request.
   * @return true if level is at least as high as the requested level.
   */
  public final boolean isConsoleLoggable(Level level) {
    CougarLogger logger = this;
    do {
      if (logger.m_consoleLevel != null) {
        return logger.m_consoleLevel.intValue() <= level.intValue();
      }
      logger = logger.m_filterParent;
    } while (logger != null);
    return false;
  }

  /**
   * Returns the logging level for the console.
   *
   * @return Current console logging level.
   *         The level will be null if the level is inherited from a parent.
   */
  public final Level getConsoleLevel() {
    return m_consoleLevel;
  }

  /**
   * Returns the logging level for handlers other than driver station console.
   *
   * @return Current logging level.
   *         The level will be null if the level is inherited from a parent.
   */
  public final Level getLevel() {
    return m_logger.getLevel();
  }

  /**
   * Get name of logger.
   *
   * @return The name is bound at the time of creation.
   */
  public final String getName() {
    return m_logger.getName();
  }

  /**
   * Change logging level for handlers other than driver station console.
   *
   * @param level The level to set or null ot inherit from parent.
   *     The parent is the logger with the prefix (prior to last '.')
   */
  public final void setLevel(Level level) {
    if (kAlwaysOnName.equals(getName())) {
      return;
    }
    m_logger.setLevel(level);
  }

  /**
   * Log a formatted message.
   *
   * <p>The arguments to the format string are only evaluated if
   * the level is loggable. Therefore you should not rely on
   * them being evaluated.
   *
   * @param level The log level.
   * @param format A String.format string to log.
   * @param args The paramters to the format string.
   */
  public void logf(Level level, String format, Object... args) {
    if (!m_logger.isLoggable(level)) {
      return;
    }

    String text = String.format(format, args);
    m_logger.log(level, text);
  }

  /**
   * A wrapper around logf for Level.FINEST
   *
   * @param format The String.format string.
   * @param args The values for the format string.
   *
   * @see #logf
   */
  public void tracef(String format, Object... args) {
    logf(Level.FINEST, format, args);
  }

  /**
   * A wrapper around logf for Level.FINE
   *
   * @param format The String.format string.
   * @param args The values for the format string.
   *
   * @see #logf
   */
  public void debugf(String format, Object... args) {
    logf(Level.FINE, format, args);
  }

  /**
   * A wrapper around logf for Level.INFO
   *
   * @param format The String.format string.
   * @param args The values for the format string.
   *
   * @see #logf
   */
  public void infof(String format, Object... args) {
    logf(Level.INFO, format, args);
  }

  /**
   * A wrapper around logf for Level.WARNING
   *
   * @param format The String.format string.
   * @param args The values for the format string.
   *
   * @see #logf
   */
  public void warningf(String format, Object... args) {
    logf(Level.WARNING, format, args);
  }

  /**
   * A wrapper around logf for Level.SEVERE
   *
   * @param format The String.format string.
   * @param args The values for the format string.
   *
   * @see #logf
   */
  public void errorf(String format, Object... args) {
    logf(Level.SEVERE, format, args);
  }

  /**
   * Change logging level for driver station console handler.
   *
   * @param level The level to set or null ot inherit from parent.
   *     The parent is the logger with the prefix (prior to last '.')
   */
  public final void setConsoleLevel(Level level) {
    if (kAlwaysOnName.equals(getName())) {
      return;
    }
    m_consoleLevel = level;

    Integer mappedLevel = level == null ? null : levelMap.get(level);
    int levelNum = mappedLevel == null ? 0 : mappedLevel.intValue();

    NetworkTable table = NetworkTableInstance.getDefault().getTable(kTable);
    table.getEntry(getName()).setInteger(levelNum);
  }

  private static void initFileHandler(Path debugLogPath) throws IOException {
    if (debugLogPath.toString().isEmpty()) {
      // Don't force initialize this handler.
      return;
    }
    // Log everything to file.
    final var root = getCougarLogger("").m_logger;

    // Ensure directory exists.
    debugLogPath.getParent().toFile().mkdir();

    _builtinFileHandler = new FileHandler(debugLogPath.toString());
    _builtinFileHandler.setLevel(Level.ALL);
    _builtinFileHandler.setFormatter(_formatter);
    root.addHandler(_builtinFileHandler);
  }

  private static void initConsoleHandler() {
    final var root = getCougarLogger("team1403").m_logger;
    java.util.logging.Handler consoleHandler = null;
    for (var handler : root.getHandlers()) {
      if (handler instanceof ConsoleHandler) {
        consoleHandler = handler;
        break;
      }
    }
    if (consoleHandler == null) {
      consoleHandler = new ConsoleHandler();
      consoleHandler.setLevel(Level.ALL);
      consoleHandler.setFormatter(_formatter);
      root.addHandler(consoleHandler);
    }

    // Create a filter for the console handler
    // That will log things if we explicitly asked them to be logged
    // to the console.
    consoleHandler.setFilter(new java.util.logging.Filter() {
        @Override
        public boolean isLoggable(LogRecord record) {
          String name = record.getLoggerName();
          CougarLogger logger;
          synchronized (_loggers) {
            logger = _loggers.get(name);
          }
          return logger.isConsoleLoggable(record.getLevel());
        }
      });
  }

  /**
   * Reset loggers for testing the CougarLogger itself.
   *
   * <p>This will discard all the loggers but components with cached references
   * will still have the old loggers.
   */
  public static void resetForTesting() {
    synchronized (_loggers) {
      java.util.logging.LogManager.getLogManager().reset();
      getCougarLogger("").m_logger.removeHandler(_builtinFileHandler);
      _builtinFileHandler = null;  // NOPMD
      _loggers.clear();
    }
  }

  /**
   * Returns standard formatter for log handler.
   *
   * @return formatter
   */
  private static java.util.logging.Formatter makeFormatter() {
    final long sysMillis = System.currentTimeMillis();
    long halMillis = HALUtil.getFPGATime() / Clock.kMicrosPerMilli;
    if (halMillis > 10 * Clock.kMillisPerSec) {
      // It appears fpgaTime is same a system time so just subtract it all.
      halMillis = 0;
    }

    // startTime is the system time where hal time starts.
    // We'll log in terms of hal time so this aligns with the datalog.
    final long startTimeMillis = sysMillis - halMillis;
    return new java.util.logging.Formatter() {
        @Override
        public String format(LogRecord record) {
          long millis = record.getMillis() - startTimeMillis;
          long secs = millis / Clock.kMillisPerSec;
          long mins = secs / 60;
          secs -= 60 * mins;
          millis -= secs * Clock.kMillisPerSec;

          return String.format("%4$c %1$02d:%2$02d.%3$03d [%5$s] %6$s%n",
                               mins, secs, millis,
                               levelMap_.getOrDefault(record.getLevel(), '?'),
                               record.getLoggerName(),
                               record.getMessage());
        }
    };
  }

  private static String generateBaseFileName() {
    final DateTimeFormatter timeFormatter =
        DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");
    return "CougarRobot_" + timeFormatter.format(LocalDateTime.now());
    
  }

  private final Logger m_logger;
  private final CougarLogger m_filterParent;  // Only used for filtering
  private Level m_consoleLevel = null;  // Level console is enabled at.

  private static Map<String, CougarLogger> _loggers = new HashMap<>(); // NOPMD

  private static final Map<Level, Character> levelMap_;

  static {
    // Initialize level symbol used in message formatter.
    levelMap_ = new HashMap<>();
    levelMap_.put(Level.SEVERE, 'E');   // error
    levelMap_.put(Level.WARNING, 'W');  // warning
    levelMap_.put(Level.INFO, 'I');     // info
    levelMap_.put(Level.FINE, 'D');     // debug
    levelMap_.put(Level.FINEST, 'T');   // trace
  }

  /**
   * The standard logging formatter used for writing log entries.
   */
  private static final java.util.logging.Formatter _formatter = makeFormatter();
  private static FileHandler _builtinFileHandler = null;

  /**
   * The basename for log files has the path but not the file extension.
   */
  public static final String kLogFileBaseName = generateBaseFileName();
}
