package team1403.lib.device.virtual;

import edu.wpi.first.networktables.NetworkTable;

import team1403.lib.device.Sensor;

/**
 * Limelight is a camera that utilizes light to detect objects on a field.
 * 
 * <p>Communication through Limelight is accomplished through network tables.
 * This class provides an Api overlaying the information provied in the tables.
 * https://docs.limelightvision.io/en/latest/networktables_api.html
 */
public class Limelight implements Sensor {
  
  /**
   * Constructor. 
   *
   * @param table Network table relaying information from limelight
   */
  public Limelight(String name, NetworkTable table) {
    m_string = name;
    m_limeTable = table;
  }
  
  @Override
  public String getName() {
    return m_string;
  }
  
  /**
   * States whether limelight has a target or not.
   *
   * @return Return whether limelight has a target.
   */
  public boolean hasTarget() {
    return m_limeTable.getEntry("tv").getDouble(0) != 0;
  }

  /**
   * Gives the horizontal offset from the middle of limelight to the target.
   *
   * @return Horizontal offset from crosshair to target
   *         (029.8 to 29.8 degrees from center).
   */
  public double getTargetDegreesX() {
    return m_limeTable.getEntry("tx").getDouble(0);
  }

  /**
   * Gives the vertical offset from the middle of limelight to the target.
   *
   * @return Vertical offset from crosshair to target
   *         (-24.85 to 24.85 degrees from center).
   */
  public double getTargetDegreesY() {
    return m_limeTable.getEntry("ty").getDouble(0);
  }

  /**
   * States how much of the target is covering the limelight's view.
   *
   * @return Target area (0 to 100 percent).
   */
  public double getTargetAreaPercentage() {
    return m_limeTable.getEntry("ta").getDouble(0);
  }

  /**
   * Return Limelight entry that isn't specified here.
   *
   * @param entryName Name of entry to get.
   * @return value of entry.
   */
  public double getTableEntry(String entryName) {
    return m_limeTable.getEntry(entryName).getDouble(0);
  }

  private final String m_string;
  private final NetworkTable m_limeTable;
}
