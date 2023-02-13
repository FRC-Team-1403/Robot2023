package preview;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Polygon;
import java.util.HashSet;
import java.util.Set;

import exceptions.SubsystemNotFound;
import util.Point;

/**
 * Represents the state of the robot for calculating and displaying the preview.
 * 
 * @author Brandon C.
 */
public class Robot {
	
	/**
	 * The initial robot to use in the preview. To make a custom subsystem display on the preview,
	 * an instance of it must be added to the initialRobot by adding it as a parameter for the varArg.
	 */
	private static final Robot initialRobot = new Robot(new Point(0, 0), 0,
		new Turret(0)
	);

	private Set<Subsystem> subsystems;
	private Point position;//in feet
	private Point robotSize;//in feet
	private double angle;//in degrees
	
	/**
	 * Create a new Robot.
	 * @param position The position of the robot relative to the field image, in feet.
	 * @param angle The angle of the robot in standard form, in degrees.
	 * @param subsystems The subsystems to use. Creates copies of the subsystems.
	 */
	public Robot(Point position, double angle, Subsystem... subsystems) {
		this.position = position.getCopy();
		this.angle = angle;
		this.robotSize = new Point(2, 3); //<-- width, height
		this.subsystems = new HashSet<Subsystem>();
		for (Subsystem s : subsystems) {
			addSubsystem(s.getCopy());
		}
	}

	/**
	 * Adds a subsystem to the Robot.
	 * @param s The subsystem to add.
	 */
	public void addSubsystem(Subsystem s) {
		subsystems.add(s);
		s.setParent(this);
	}

	/**
	 * Gets a subsystem from the set.
	 * @param name The name of the subsystem
	 * @return The subsystem with the given name. null if not found.
	 */
	public Subsystem getSubsystem(String name) {
		for (Subsystem s : subsystems) {
			if (s.getName().equals(name)) {
				return s;
			}
		}
		throw new SubsystemNotFound(name + ", available subsystems: " + subsystems.toString());
	}

	/**
	 * Gets the position of the robot.
	 * @return The position.
	 */
	public Point getPosition() {
		return position.getCopy();
	}
	
	/**
	 * Sets the position of the robot.
	 * @param p The position to use.
	 */
	public void setPosition(Point p) {
		this.position = p.getCopy();
	}
	
	/**
	 * Gets the angle of the robot.
	 * @return The angle in degrees.
	 */
	public double getAngle() {
		return angle;
	}
	
	/**
	 * Sets the angle of the robot.
	 * @param angle The angle in degrees.
	 */
	public void setAngle(double angle) {
		this.angle = angle;
	}
	
	/**
	 * Gets a copy of the robot with the same position, angle, and subsystems.
	 */
	public Robot getCopy() {
		Robot copy = new Robot(getPosition(), angle);
		for (Subsystem s : subsystems) {
			copy.addSubsystem(s.getCopy());
		}
		return copy;
	}
	
	/**
	 * Gets the corners of the robot.
	 * @param size The size of hte rectangle to use.
	 * @return An array of length 4 containing the 4 corners of the robot.
	 */
	public Point[] getCorners(Point size) {
		Point[] corners = new Point[4];//tl, tr, br, bl
		corners[0] = new Point(-size.x/2, size.y/2);
		corners[1] = new Point(size.x/2, size.y/2);
		corners[2] = new Point(size.x/2, -size.y/2);
		corners[3] = new Point(-size.x/2, -size.y/2);
		for (int i = 0; i < corners.length; i++) {
			corners[i] = corners[i].getRotatedAroundPoint(Point.origin, angle).add(position); 
		}
		return corners;
	}
	
	/**
	 * Draws the robot and its subsystems.
	 * @param g The graphics object to use.
	 */
	public void draw(Graphics g) {
		Point[] corners = getCorners(robotSize);
		Polygon p = new Polygon();
		for (int i = 0; i < corners.length; i++) {
			corners[i] = corners[i].toPixels(); 
			p.addPoint((int)corners[i].x, (int)corners[i].y);
		}
		g.setColor(Color.gray);
		g.fillPolygon(p);
		g.setColor(Color.black);
		g.drawPolygon(p);
		g.setColor(Color.green);
		Point endOfAngle = Point.fromPolar(25, angle).add(position.toPixels());
		position.toPixels().drawLine(g, endOfAngle);
		for (Subsystem s : subsystems) {
			s.draw(g);
		}
	}

	/**
	 * Gets a copy of the initial robot, including its subsystems.
	 * @return A copy of the initial robot.
	 */
	public static Robot getInitialRobot() {
		return initialRobot.getCopy();
	}
	
}
