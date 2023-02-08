package util;

import java.awt.Graphics;
import java.math.RoundingMode;
import java.text.DecimalFormat;

import gui.MainWindow;
import gui_elements.Button;

/**
 * Utility class for storing a position with double precision.
 * 
 * @author Brandon C.
 */
public class Point {
	
	public static final Point origin = new Point(0, 0);
	
	public double x;
	public double y;
	
	/**
	 * Create a new point.
	 * @param x The x position of the point.
	 * @param y The y position of the point.
	 */
	public Point (double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Create a new point.
	 * @param p The position of the point.
	 */
	public Point (Pos p) {
		this.x = p.x;
		this.y = p.y;
	}

	/**
	 * Gets a point from its polar coordinates.
	 * @param distance The distancae from the origin.
	 * @param angle The angle in degrees between the x axis and the origin.
	 * @return A new point from the specified polar coordinates.
	 */
	public static Point fromPolar(double distance, double angle) {
		double cx = distance * Math.cos(Math.toRadians(angle += 90));
		double cy = -1 * distance * Math.sin(Math.toRadians(angle));
		return new Point(cx, cy);
	}
	
	/**
	 * Gets a new point with the same coordinates
	 * @return A new point using the x and y.
	 */
	public Point getCopy() {
		return new Point(x, y);
	}
	
	/**
	 * Draws a dot on the screen that represents the position of the point.
	 * @param g The graphics object to draw with.
	 */
	public void draw(Graphics g) {
		g.fillOval((int) x - 3, (int) y - 3, 6, 6);
	}
	
	/**
	 * Draws a line between the point and another point.
	 * @param g The graphics object to draw with.
	 * @param other The other point to draw a line to.
	 */
	public void drawLine(Graphics g, Point other) {
		g.drawLine((int) x, (int) y, (int) other.x, (int) other.y);
	}
	
	/**
	 * Gets a new point with the current position scaled based on the preview conversion.
	 * @return New point with position in feet relative to the center of the preview.
	 */
	public Point toFeet() {
		MainWindow mw = MainWindow.getInstance();
		Button img = mw.imageRect;
		double scale = mw.getFieldImageScale();
		return new Point((img.x + img.w - x)/scale, (img.y - y + img.h)/scale);
	}
	
	/**
	 * Gets a new point with the current position scaled based on the preview conversion.
	 * @return New point with position in screen space pixels relative to the top left corner of the screen.
	 */
	public Point toPixels() {
		MainWindow mw = MainWindow.getInstance();
		Button img = mw.imageRect;
		double scale = mw.getFieldImageScale();
		return new Point(img.x + img.w -x*scale, ((img.y / scale) - y + (img.h / scale))*scale);
	}
	
	/**
	 * Gets a new point based on the current point translated by the other point.
	 * @param p The point to add.
	 * @return A new point with position added.
	 */
	public Point add(Point p) {
		return new Point(x+p.x, y+p.y);
	}
	
	/**
	 * Gets the angle between the x axis and the line from the point to the other point in degrees.
	 * @param other The point to use as the endpoint of the line.
	 * @return The angle in degrees.
	 */
	public double getAngleToPoint(Point other) {
		return Math.toDegrees(Math.atan2(other.x - x, other.y - y));
	}
	
	/**
	 * Gets the distance between the point and another point.
	 * @param other The other point.
	 * @return The distance between the two points.
	 */
	public double getDistance(Point other) {
		return Math.sqrt(Math.pow(other.x - x, 2) + Math.pow(other.y - y, 2));
	}
	
	/**
	 * Gets a new point with the position of the current point rotated around a pivot point.
	 * @param pivot The point to rotate around.
	 * @param angle The angle to rotate by in degrees.
	 * @return A new point which is rotated around the pivot.
	 */
	public Point getRotatedAroundPoint(Point pivot, double angle) {
		double currentAngle = pivot.getAngleToPoint(this);
		double distance = getDistance(pivot);
		currentAngle -= angle;
		Point rotated = fromPolar(distance, currentAngle);
		return rotated.add(pivot);
	}
	
	/**
	 * Gets a new point with the position of the point to 2 decimal places.
	 * @return A new point with rounded position.
	 */
	public Point getRounded() {
		DecimalFormat df = new DecimalFormat("#.##");
		df.setRoundingMode(RoundingMode.CEILING);
		return new Point(Double.parseDouble(df.format(x)), Double.parseDouble(df.format(y)));
	}
}

