package preview;

import java.awt.Graphics;

/**
 * Can be added to the robot to give the preview additional functionality.
 * 
 * @author Brandon C.
 */
public abstract class Subsystem {

    private String name;
    private Robot parent;

    /**
     * Create a new Subsystem.
     * @param name The name of the subsystem.
     */
    public Subsystem(String name) {
        this.name = name;
    }

    /**
     * Sets the parent robot of the Subsystem.
     * @param parent The robot to set as the parent.
     */
    public void setParent(Robot parent) {
        this.parent = parent;
    }

    /**
     * Gets the parent robot of the subsystem.
     * @return The parent robot.
     */
    public Robot getParent() {
        return parent;
    }

    /**
     * Gets the name of the Subsystem.
     * @return The name of the subsystem.
     */
    public String getName() {
        return name;
    }

    /**
     * Draws the Subsystem on the preview.
     * @param g The graphics object to use.
     */
    public abstract void draw(Graphics g);

    /**
     * Gets a copy of the Subsystem.
     * @return Subsystem A subsystem with the same information.
     */
    public abstract Subsystem getCopy();
}