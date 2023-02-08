package util;

import java.awt.event.MouseEvent;

/**
 * Utility class for storing a position with integer precision.
 * 
 * @author Brandon C.
 */
public class Pos {

	public static final Pos ORIGIN = new Pos(0, 0);

    public int x;
	public int y;
	
	/**
	 * Create a new Pos
	 * @param x The x position.
	 * @param y The y position.
	 */
	public Pos (int x, int y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Create a new Pos.
	 * @param e MouseEvent to use as position.
	 */
	public Pos(MouseEvent e) {
		this(e.getX(), e.getY());
	}

	/**
	 * Gets a new Pos with the position translated.
	 * @param other The amount to translate by.
	 * @return New Pos with translated(added) position.
	 */
	public Pos add(Pos other) {
		return new Pos(x + other.x, y + other.y);
	}

	/**
	 * Gets a new Pos with the position translated in the opposite direction.
	 * @param other The amount to translate by.
	 * @return New Pos with translated(added) position.
	 */
	public Pos subtract(Pos other) {
		return new Pos(x - other.x, y - other.y);
	}
}