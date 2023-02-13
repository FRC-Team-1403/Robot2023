package team1403.lib.util;

/**
 * Class to store dimensions of an object.
 * Stores 3d dimensions, the 3rd dimension can be null.
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

  public double getLength() {
    return m_length;
  }

  public double getWidth() {
    return m_width;
  }

  public double getDepth() {
    return m_depth;
  }

}
