package team1403.lib.util;

/**
 * Sets the methods for .
 */
public class Dimension {
  private final double m_length;
  private final double m_width;
  private final double m_depth;

  /**
   * Dimensions for arm.
   *
   * @param height height of arm
   * @param width width of arm
   */
  public Dimension(double height, double width) {
    this.m_length = height;
    this.m_width = width;
    this.m_depth = 0;
  }

  /**
   * Dimensions for arm.
   *
   * @param height height of arm
   * @param width width of arm
   * @param depth depth of arm
   */
  public Dimension(double height, double width, double depth) {
    this.m_length = height;
    this.m_width = width;
    this.m_depth = depth;
  }

  public double getM_length() {
    return m_length;
  }

  public double getM_width() {
    return m_width;
  }

  public double getM_depth() {
    return m_depth;
  }

}
