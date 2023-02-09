package preview;

import java.awt.Color;
import java.awt.Graphics;

import util.Point;

/**
 * A subsystem implementation for the Turret on the 1403 2021 robot.
 * 
 * @author Brandon C.
 */
public class Turret extends Subsystem {

    private double turretAngle;

    /**
     * Create a new Turret.
     * @param turretAngle The angle of the turret.
     */
    public Turret(double turretAngle) {
        super("Turret");
        this.turretAngle = turretAngle;
    }

    /**
     * Sets the angle of the turret relative to the robot.
     * @param angle The angle of the turret in degrees.
     */
    public void setAngle(double angle) {
        turretAngle = angle;
    }

    /**
     * {@inheritDoc} Draws a magenta line to represent the turret.
     */
    @Override
    public void draw(Graphics g) {
        g.setColor(Color.magenta);
        Point position = getParent().getPosition();
        Point endOfTurret = Point.fromPolar(50, 
            turretAngle + getParent().getAngle())
            .add(position.toPixels());
		position.toPixels().drawLine(g, endOfTurret);
    }

    @Override
    public Subsystem getCopy() {
        return new Turret(this.turretAngle);
    }
}